/*
 * Teste de host do planejador de movimento (main/motion_profile.c).
 * Executa o gerador de simbolos exatamente como o ISR do RMT e mede o resultado fisico.
 *
 * Compilar/rodar (sem ESP-IDF):  python tests/host/run_host_tests.py
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "motion_profile.h"

#define TICK_HZ 1000000U
#define POOL_WORDS 8192U

static uint32_t g_pool[POOL_WORDS];
static int g_failures = 0;

#define CHECK(cond, ...)                                                        \
    do {                                                                        \
        if (!(cond)) {                                                          \
            printf("  FALHOU (%s:%d): ", __FILE__, __LINE__);                   \
            printf(__VA_ARGS__);                                                \
            printf("\n");                                                       \
            g_failures++;                                                       \
        }                                                                       \
    } while (0)

typedef struct {
    uint32_t steps;
    uint64_t end_ticks;        /* duracao total emitida no canal */
    double *step_time;         /* instante (s) de cada borda de subida */
    size_t symbols;
    int bad_symbols;
} emission_t;

/* Consome o programa como o RMT: soma duracoes, registra a borda de subida de cada pulso */
static emission_t emit(const mp_axis_prog_t *prog, size_t chunk)
{
    emission_t e = {0};
    e.step_time = calloc(prog->total_steps + 1U, sizeof(double));
    mp_gen_t g;
    mp_gen_reset(&g, prog);
    uint32_t words[64];
    bool done = false;
    uint64_t t = 0;
    while (!done) {
        size_t n = mp_gen_fill(&g, words, chunk, &done);
        for (size_t i = 0; i < n; ++i) {
            uint32_t w = words[i];
            uint32_t d0 = w & 0x7FFFU, l0 = (w >> 15) & 1U, d1 = (w >> 16) & 0x7FFFU, l1 = (w >> 31) & 1U;
            if (d0 == 0U || d1 == 0U || l0 != 0U) {
                e.bad_symbols++;
            }
            t += d0;
            if (l1) {
                e.step_time[e.steps++] = (double)t / TICK_HZ;
            }
            t += d1;
            e.symbols++;
        }
        if (n == 0U && !done) {
            e.bad_symbols++;
            break;
        }
    }
    e.end_ticks = t;
    return e;
}

/*
 * Maior aceleracao observada (passos/s^2). A velocidade e medida em janelas de W passos:
 * com ticks de 1 us o periodo em alta velocidade alterna (ex.: 156/157 ticks, dithering da
 * fracao Q8) e a derivada passo a passo mediria esse ruido, nao a aceleracao do perfil.
 */
static double max_accel(const emission_t *e)
{
    const uint32_t W = 12;
    double amax = 0.0;
    if (e->steps < 3 * W) {
        return 0.0;
    }
    for (uint32_t k = 0; k + 2 * W < e->steps; ++k) {
        double t0 = e->step_time[k], t1 = e->step_time[k + W], t2 = e->step_time[k + 2 * W];
        double v1 = W / (t1 - t0), v2 = W / (t2 - t1);
        double a = fabs(v2 - v1) / (0.5 * (t2 - t0));
        if (a > amax) {
            amax = a;
        }
    }
    return amax;
}

static mp_request_t req3(int32_t c, int32_t a, int32_t z)
{
    mp_request_t r;
    memset(&r, 0, sizeof(r));
    r.steps[0] = c;
    r.steps[1] = a;
    r.steps[2] = z;
    for (int i = 0; i < 3; ++i) {
        r.vmax[i] = (i == 2) ? 20000.0f : 6400.0f;     /* 720 deg/s @3200 passos/volta; 250 mm/s */
        r.accel[i] = (i == 2) ? 80000.0f : 16000.0f;   /* 1800 deg/s^2; 1000 mm/s^2 */
        r.v_floor[i] = (i == 2) ? 1200.0f : 89.0f;     /* RAMP 10 deg/s; 15 mm/s */
    }
    return r;
}

static void test_single_axis_long_move(void)
{
    printf("[single_axis_long_move]\n");
    mp_request_t r = req3(12800, 0, 0); /* 4 voltas */
    mp_plan_t p;
    CHECK(mp_plan_move(&r, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "plan");
    emission_t e = emit(&p.axis[0], 24);
    CHECK(e.bad_symbols == 0, "simbolos invalidos: %d", e.bad_symbols);
    CHECK(e.steps == 12800U, "passos emitidos %u", e.steps);
    CHECK(e.end_ticks == p.duration_ticks, "duracao %llu != %u", (unsigned long long)e.end_ticks, p.duration_ticks);
    double amax = max_accel(&e);
    CHECK(amax <= 16000.0 * 1.08, "aceleracao max %.0f > 16000", amax);
    CHECK(amax >= 16000.0 * 0.85, "aceleracao max %.0f nao chega perto do limite", amax);
    double v_cruise = 1.0 / (e.step_time[6400] - e.step_time[6399]);
    CHECK(fabs(v_cruise - 6400.0) < 6400.0 * 0.02, "cruzeiro %.1f != 6400", v_cruise);
    double v_start = 1.0 / (e.step_time[1] - e.step_time[0]);
    CHECK(v_start < 300.0, "partida %.1f passos/s (esperado perto de 89)", v_start);
    /* Idle axes: canal so de espera com a mesma duracao */
    emission_t ea = emit(&p.axis[1], 24);
    CHECK(ea.steps == 0U && ea.end_ticks == p.duration_ticks, "eixo parado desalinhado");
    printf("  duracao %.3f s, acel max %.0f, cruzeiro %.0f, partida %.0f passos/s, pool %zu\n",
           p.duration_s, amax, v_cruise, v_start, p.pool_used);
    free(e.step_time);
    free(ea.step_time);
}

static void test_short_triangular_move(void)
{
    printf("[short_triangular_move]\n");
    mp_request_t r = req3(-400, 0, 0); /* 45 graus, nao alcanca 720 deg/s */
    mp_plan_t p;
    CHECK(mp_plan_move(&r, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "plan");
    CHECK(!p.axis[0].dir_positive, "sentido");
    emission_t e = emit(&p.axis[0], 7);
    CHECK(e.steps == 400U, "passos %u", e.steps);
    double amax = max_accel(&e);
    CHECK(amax <= 16000.0 * 1.08, "aceleracao max %.0f", amax);
    CHECK(p.v_cruise < 6400.0f, "pico deveria cair: %.0f", p.v_cruise);
    printf("  pico %.0f passos/s em %.3f s, acel max %.0f\n", p.v_cruise, p.duration_s, amax);
    free(e.step_time);
}

static void test_sync_axes_stay_coordinated(void)
{
    printf("[sync_axes_coordinated]\n");
    mp_request_t r = req3(3200, -800, 4000);
    mp_plan_t p;
    CHECK(mp_plan_move(&r, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "plan");
    emission_t ec = emit(&p.axis[0], 24), ea = emit(&p.axis[1], 24), ez = emit(&p.axis[2], 24);
    CHECK(ec.steps == 3200U && ea.steps == 800U && ez.steps == 4000U, "passos %u %u %u", ec.steps, ea.steps, ez.steps);
    CHECK(ec.end_ticks == ea.end_ticks && ea.end_ticks == ez.end_ticks, "fim desalinhado");
    /* Z e dominante: o passo k de C ocorre quando Z esta em k*4000/3200 */
    double worst = 0.0;
    for (uint32_t k = 1; k < 3200; k += 97) {
        double ratio = 4000.0 / 3200.0;
        double s_dom = ((double)k + 0.5) * ratio - 0.5;
        uint32_t kz = (uint32_t)floor(s_dom);
        double frac = s_dom - kz;
        if (kz + 1 >= ez.steps) break;
        double tz = ez.step_time[kz] + frac * (ez.step_time[kz + 1] - ez.step_time[kz]);
        double err = fabs(ec.step_time[k] - tz);
        if (err > worst) worst = err;
    }
    CHECK(worst < 200e-6, "desvio de coordenacao %.1f us", worst * 1e6);
    CHECK(max_accel(&ec) <= 16000.0 * 1.08, "C excede aceleracao: %.0f", max_accel(&ec));
    CHECK(max_accel(&ez) <= 80000.0 * 1.08, "Z excede aceleracao: %.0f", max_accel(&ez));
    printf("  duracao %.3f s, desvio max de coordenacao %.1f us\n", p.duration_s, worst * 1e6);
    free(ec.step_time);
    free(ea.step_time);
    free(ez.step_time);
}

static void test_tiny_and_edge_moves(void)
{
    printf("[tiny_and_edge_moves]\n");
    int32_t sizes[] = {1, 2, 3, 5, 9, 24, 81};
    for (size_t i = 0; i < sizeof(sizes) / sizeof(sizes[0]); ++i) {
        mp_request_t r = req3(sizes[i], sizes[i] > 3 ? 1 : 0, 0);
        mp_plan_t p;
        CHECK(mp_plan_move(&r, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "plan %d", sizes[i]);
        emission_t e = emit(&p.axis[0], 3), e2 = emit(&p.axis[1], 3);
        CHECK(e.steps == (uint32_t)sizes[i], "passos %u != %d", e.steps, sizes[i]);
        CHECK(e.bad_symbols == 0 && e2.bad_symbols == 0, "simbolos invalidos em %d", sizes[i]);
        CHECK(e.end_ticks == e2.end_ticks, "fim desalinhado em %d", sizes[i]);
        free(e.step_time);
        free(e2.step_time);
    }
    mp_request_t empty = req3(0, 0, 0);
    mp_plan_t p;
    CHECK(mp_plan_move(&empty, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_ERR_EMPTY, "vazio");
    /* Movimento muito lento: periodos > 32767 ticks exigem simbolos de preenchimento */
    mp_request_t slow = req3(5, 0, 0);
    slow.vmax[0] = 10.0f;
    slow.v_floor[0] = 5.0f;
    CHECK(mp_plan_move(&slow, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "lento");
    emission_t e = emit(&p.axis[0], 2);
    CHECK(e.steps == 5U && e.bad_symbols == 0, "lento: %u passos, %d invalidos", e.steps, e.bad_symbols);
    free(e.step_time);
}

static void test_pool_limit_reduces_speed(void)
{
    printf("[pool_limit]\n");
    mp_request_t r = req3(200000, 0, 0);
    r.accel[0] = 500.0f; /* rampa longuissima */
    mp_plan_t p;
    uint32_t small[1024];
    mp_result_t res = mp_plan_move(&r, small, 1024, TICK_HZ, &p);
    CHECK(res == MP_OK, "deveria reduzir o cruzeiro: %d", res);
    CHECK(p.pool_used <= 1024U, "pool %zu", p.pool_used);
    CHECK(p.v_cruise < 6400.0f, "cruzeiro nao reduzido");
    printf("  cruzeiro reduzido para %.0f passos/s (pool %zu/1024)\n", p.v_cruise, p.pool_used);
}

static void test_junction_and_chained_moves(void)
{
    printf("[junction_chain]\n");
    float jerk[3] = {133.0f, 133.0f, 800.0f};
    mp_request_t a = req3(3200, 0, 0), b = req3(3200, 0, 0), rev = req3(-3200, 0, 0), other = req3(0, 3200, 0);
    float xa = 0.0f, yb = 0.0f;
    CHECK(mp_junction(&a, &b, jerk, &xa, &yb), "mesmo sentido deveria encadear");
    CHECK(fabsf(xa * 3200.0f - 6400.0f) < 1.0f, "juncao colinear deveria manter o cruzeiro: %.1f", xa * 3200.0f);
    CHECK(!mp_junction(&a, &rev, jerk, &xa, &yb), "reversao deve parar");
    CHECK(!mp_junction(&a, &other, jerk, &xa, &yb), "troca de eixo acima do jerk deve parar");

    /* Execucao encadeada: saida de a = entrada de b, sem desacelerar entre elas */
    mp_junction(&a, &b, jerk, &xa, &yb);
    a.rate_exit = xa;
    b.rate_entry = yb;
    mp_plan_t pa, pb;
    static uint32_t pool2[POOL_WORDS];
    CHECK(mp_plan_move(&a, g_pool, POOL_WORDS, TICK_HZ, &pa) == MP_OK, "plan a");
    CHECK(mp_plan_move(&b, pool2, POOL_WORDS, TICK_HZ, &pb) == MP_OK, "plan b");
    emission_t ea = emit(&pa.axis[0], 24), eb = emit(&pb.axis[0], 24);
    double last_a = 1.0 / (ea.step_time[3199] - ea.step_time[3198]);
    double first_b = 1.0 / (eb.step_time[1] - eb.step_time[0]);
    CHECK(fabs(last_a - first_b) < 150.0, "salto na juncao %.0f -> %.0f", last_a, first_b);
    /* Intervalo real que cruza a juncao (fim de a + inicio de b) */
    double gap = ((double)ea.end_ticks / TICK_HZ - ea.step_time[3199]) + eb.step_time[0];
    CHECK(fabs(1.0 / gap - 6400.0) < 6400.0 * 0.05, "passo na juncao a %.0f passos/s", 1.0 / gap);
    printf("  saida a %.0f, entrada b %.0f, passo da juncao %.0f passos/s\n", last_a, first_b, 1.0 / gap);
    free(ea.step_time);
    free(eb.step_time);
}

/* Segmento do jog: v_floor = vmax -> velocidade constante, duracao exata, eixos alinhados */
static void test_jog_constant_velocity_segment(void)
{
    const int32_t cases[][3] = {{12, -3, 0}, {1, 0, 0}, {0, 7, 40}, {1, 1, 1}, {125, 0, -2}};
    const double seg_s[] = {0.010, 0.010, 0.020, 0.080, 0.010};
    for (size_t k = 0; k < sizeof(cases) / sizeof(cases[0]); ++k) {
        mp_request_t r;
        memset(&r, 0, sizeof(r));
        for (int i = 0; i < 3; ++i) {
            float v = fabsf((float)cases[k][i]) / (float)seg_s[k];
            r.steps[i] = cases[k][i];
            r.vmax[i] = v;
            r.v_floor[i] = v;
            r.accel[i] = 1e9f;
        }
        mp_plan_t p;
        CHECK(mp_plan_move(&r, g_pool, POOL_WORDS, TICK_HZ, &p) == MP_OK, "plan jog %zu", k);
        uint32_t expect = (uint32_t)(seg_s[k] * TICK_HZ + 0.5);
        CHECK(p.duration_ticks >= expect - 2U && p.duration_ticks <= expect + 2U,
              "jog %zu: duracao %u ticks, esperado %u", k, p.duration_ticks, expect);
        for (int i = 0; i < 3; ++i) {
            emission_t e = emit(&p.axis[i], 24);
            uint32_t want = (uint32_t)abs(cases[k][i]);
            CHECK(e.steps == want && e.bad_symbols == 0, "jog %zu eixo %d: %u passos, %d invalidos",
                  k, i, e.steps, e.bad_symbols);
            CHECK(e.end_ticks == p.duration_ticks, "jog %zu eixo %d desalinhado", k, i);
            if (want >= 3U) {
                /* Espacamento uniforme: velocidade constante dentro do segmento */
                double d0 = e.step_time[1] - e.step_time[0];
                double d1 = e.step_time[want - 1] - e.step_time[want - 2];
                CHECK(fabs(d0 - d1) < 3e-6, "jog %zu eixo %d nao uniforme: %.1f vs %.1f us", k, i, d0 * 1e6, d1 * 1e6);
            }
            free(e.step_time);
        }
    }
}

/* ---------------------------------------------------------------------------------- */
/* Seguidor em tempo real (jog / streaming U)                                          */
/* ---------------------------------------------------------------------------------- */

/* Eixo C padrao: 200 passos x 16 micro = 8,889 passos/grau; defaults novos do firmware */
#define TRK_SPD 8.8889f
#define TRK_DT 0.005f
#define TRK_ACC (3600.0f * TRK_SPD)
#define TRK_JERK (180000.0f * TRK_SPD)

static mp_track_limits_t trk_limits(void)
{
    mp_track_limits_t l = {.vmax = 720.0f * TRK_SPD, .accel = TRK_ACC,
                           .smooth_n = mp_track_smooth_periods(TRK_ACC, TRK_JERK, TRK_DT)};
    return l;
}

/* Atraso da media movel, compensado pelo firmware avaliando o alvo a frente */
static float trk_delay(const mp_track_limits_t *l)
{
    return 0.5f * (float)(l->smooth_n - 1U) * TRK_DT;
}

typedef struct {
    double max_overshoot;
    double max_acc;
    double max_jerk;
    double max_vel;
    double final_err;
    int settle_k;
} trk_stats_t;

static trk_stats_t trk_run_step(float target, float seconds)
{
    mp_track_limits_t lim = trk_limits();
    mp_track_state_t s;
    mp_track_reset(&s, 0.0f);
    trk_stats_t st = {0};
    st.settle_k = -1;
    float prev_acc = 0.0f;
    int n = (int)(seconds / TRK_DT);
    for (int k = 0; k < n; ++k) {
        mp_track_step(&s, &lim, target, 0.0f, TRK_DT);
        double over = (target >= 0.0f) ? s.pos - target : target - s.pos;
        if (over > st.max_overshoot) st.max_overshoot = over;
        if (fabs(s.acc) > st.max_acc) st.max_acc = fabs(s.acc);
        if (fabs(s.vel) > st.max_vel) st.max_vel = fabs(s.vel);
        double jerk = fabs(s.acc - prev_acc) / TRK_DT;
        if (jerk > st.max_jerk) st.max_jerk = jerk;
        prev_acc = s.acc;
        if (st.settle_k < 0 && mp_track_idle(&s)) st.settle_k = k + 1;
    }
    st.final_err = fabs(s.pos - target);
    return st;
}

static void test_track_step_no_overshoot(void)
{
    mp_track_limits_t lim = trk_limits();
    double jerk_lim = TRK_JERK;
    const float targets[] = {1.0f, 5.0f, 80.0f, 800.0f, -3200.0f, 9000.0f};
    for (size_t k = 0; k < sizeof(targets) / sizeof(targets[0]); ++k) {
        trk_stats_t st = trk_run_step(targets[k], 4.0f);
        printf("[track_step %g passos N=%u] overshoot %.4f  v %.0f  a %.0f/%.0f  jerk %.0f/%.0f  parado em %d ms\n",
               targets[k], lim.smooth_n, st.max_overshoot, st.max_vel, st.max_acc, lim.accel, st.max_jerk,
               jerk_lim, st.settle_k * 5);
        CHECK(st.final_err < 1e-3, "degrau %g nao chegou (erro %g)", targets[k], st.final_err);
        CHECK(st.settle_k > 0, "degrau %g nao ficou ocioso", targets[k]);
        CHECK(st.max_overshoot < 0.01, "degrau %g ultrapassou %g passos", targets[k], st.max_overshoot);
        CHECK(st.max_vel <= lim.vmax * 1.001, "degrau %g excedeu vmax", targets[k]);
        /* o pouso do seguidor interno pode somar uma fracao de periodo de aceleracao */
        CHECK(st.max_acc <= lim.accel * 1.05, "degrau %g excedeu accel (%g)", targets[k], st.max_acc);
        CHECK(st.max_jerk <= jerk_lim * 1.01, "degrau %g excedeu jerk (%g)", targets[k], st.max_jerk);
    }
}

/* Alvo continuo com velocidade exata: rampa e senoide, com e sem feedforward */
static void test_track_feedforward_continuous(void)
{
    mp_track_limits_t lim = trk_limits();
    float lead = trk_delay(&lim);
    {
        mp_track_state_t s;
        mp_track_reset(&s, 0.0f);
        float v = 200.0f * TRK_SPD;
        double max_err = 0.0;
        for (int k = 1; k <= 400; ++k) {
            float t = k * TRK_DT; /* fim do periodo; o alvo passado e o do inicio (+ lead) */
            mp_track_step(&s, &lim, v * (t - TRK_DT + lead), v, TRK_DT);
            if (t > 0.3f && fabs(s.pos - v * t) > max_err) max_err = fabs(s.pos - v * t);
        }
        printf("[track_ramp 200 deg/s] erro apos 0,3 s: %.3f passos\n", max_err);
        CHECK(max_err < 1.0, "rampa com feedforward atrasada %.2f passos", max_err);
    }
    for (int ff = 0; ff <= 1; ++ff) {
        mp_track_state_t s;
        mp_track_reset(&s, 0.0f);
        float amp = 45.0f * TRK_SPD;
        float w = 2.0f * 3.14159265f;
        double max_err = 0.0;
        for (int k = 1; k <= 800; ++k) {
            float t = k * TRK_DT;
            float t0 = t - TRK_DT + (ff ? lead : 0.0f);
            mp_track_step(&s, &lim, amp * sinf(w * t0), ff ? amp * w * cosf(w * t0) : 0.0f, TRK_DT);
            float x = amp * sinf(w * t);
            if (t > 1.0f && fabs(s.pos - x) > max_err) max_err = fabs(s.pos - x);
        }
        printf("[track_sine 45 deg 1 Hz %s feedforward] erro max %.2f passos (%.2f deg)\n",
               ff ? "com" : "sem", max_err, max_err / TRK_SPD);
        if (ff) {
            CHECK(max_err < 2.0 * TRK_SPD, "senoide com feedforward: erro %.2f passos", max_err);
        }
    }
}

/* Alvo amostrado como o TouchDesigner manda: frames a taxa fixa, chegada com jitter */
static void test_track_sampled_stream(void)
{
    mp_track_limits_t lim = trk_limits();
    float lead = TRK_DT + trk_delay(&lim); /* firmware: segmento em voo + media movel */
    const float rates[] = {30.0f, 60.0f, 120.0f};
    for (size_t r = 0; r < sizeof(rates) / sizeof(rates[0]); ++r) {
        for (int ff = 0; ff <= 1; ++ff) {
            mp_track_state_t s;
            mp_track_reset(&s, 0.0f);
            mp_track_feed_t feed;
            mp_track_feed_reset(&feed);
            float amp = 90.0f * TRK_SPD;
            float w = 2.0f * 3.14159265f * 0.5f;
            int64_t frame_us = (int64_t)(1e6f / rates[r]);
            int64_t next_frame = 0;
            unsigned seed = 12345U;
            double max_err = 0.0, max_jerk = 0.0;
            float prev_acc = 0.0f;
            for (int k = 1; k <= 1600; ++k) {
                int64_t now = (int64_t)k * 5000;
                while (next_frame <= now) {
                    seed = seed * 1103515245U + 12345U;
                    int64_t jitter = (int64_t)((seed >> 16) % 2000U); /* 0..2 ms USB + CAN */
                    float x = amp * sinf(w * (float)next_frame * 1e-6f);
                    mp_track_feed_update(&feed, x, next_frame + jitter);
                    next_frame += frame_us;
                }
                float tgt, vel;
                mp_track_feed_eval(&feed, now, ff ? lead : 0.0f, &tgt, &vel);
                mp_track_step(&s, &lim, ff ? tgt : feed.target, ff ? vel : 0.0f, TRK_DT);
                /* o segmento planejado agora roda depois do que ja esta em voo: termina em now + 2 dt */
                float truth = amp * sinf(w * (float)(now + 2 * 5000) * 1e-6f);
                if (k > 400) {
                    if (fabs(s.pos - truth) > max_err) max_err = fabs(s.pos - truth);
                    double j = fabs(s.acc - prev_acc) / TRK_DT;
                    if (j > max_jerk) max_jerk = j;
                }
                prev_acc = s.acc;
            }
            printf("[track_stream %.0f Hz %s feedforward] erro max %.2f deg  jerk max %.0f deg/s3\n",
                   rates[r], ff ? "com" : "sem", max_err / TRK_SPD, max_jerk / TRK_SPD);
            if (ff) {
                /* extrapolacao linear: o erro cresce com o intervalo entre frames */
                double lim_deg = (rates[r] < 45.0f) ? 3.0 : 1.5;
                CHECK(max_err / TRK_SPD < lim_deg, "stream %.0f Hz: erro %.2f deg", rates[r], max_err / TRK_SPD);
            }
        }
    }
}

static void test_track_feed_gaps(void)
{
    mp_track_feed_t f;
    mp_track_feed_reset(&f);
    float tgt, vel;
    mp_track_feed_update(&f, 0.0f, 0);
    mp_track_feed_update(&f, 10.0f, 16667);
    mp_track_feed_eval(&f, 16667 + 8000, 0.0f, &tgt, &vel);
    CHECK(fabsf(vel - 600.0f) < 1.0f && fabsf(tgt - 14.8f) < 0.2f, "extrapolacao: alvo %g vel %g", tgt, vel);
    /* Sem frame novo alem do horizonte: alvo para, velocidade zero */
    mp_track_feed_eval(&f, 16667 + 100000, 0.0f, &tgt, &vel);
    CHECK(vel == 0.0f && tgt < 10.0f + 600.0f * 0.0209f + 0.1f, "horizonte: alvo %g vel %g", tgt, vel);
    /* Lacuna longa (TD pausado): nao gera velocidade gigante */
    mp_track_feed_update(&f, 500.0f, 16667 + 900000);
    mp_track_feed_eval(&f, 16667 + 905000, 0.02f, &tgt, &vel);
    CHECK(vel == 0.0f && tgt == 500.0f, "lacuna: alvo %g vel %g", tgt, vel);
    /* Alvo parado (TD continua mandando o mesmo ponto): velocidade zero no frame seguinte */
    mp_track_feed_update(&f, 500.0f, 16667 + 921667);
    mp_track_feed_update(&f, 500.0f, 16667 + 938334);
    mp_track_feed_eval(&f, 16667 + 945000, 0.02f, &tgt, &vel);
    CHECK(vel == 0.0f && tgt == 500.0f, "parado: alvo %g vel %g", tgt, vel);
}

int main(void)
{
    test_track_step_no_overshoot();
    test_track_feedforward_continuous();
    test_track_sampled_stream();
    test_track_feed_gaps();
    test_jog_constant_velocity_segment();
    test_single_axis_long_move();
    test_short_triangular_move();
    test_sync_axes_stay_coordinated();
    test_tiny_and_edge_moves();
    test_pool_limit_reduces_speed();
    test_junction_and_chained_moves();
    if (g_failures) {
        printf("\n%d verificacao(oes) falharam\n", g_failures);
        return 1;
    }
    printf("\nTodos os testes de host passaram\n");
    return 0;
}
