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

int main(void)
{
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
