#include "motion_profile.h"

#include <math.h>
#include <string.h>

typedef struct {
    uint8_t dom;
    uint32_t n;            /* passos do dominante */
    float r[MP_AXES];      /* N_i / N_dom */
    float vmax;            /* limites no eixo dominante */
    float accel;
    float vfloor;
} mp_path_t;

typedef struct {
    float v0, vc, v1, A;
    float Ta, Tc, Td;
    float Sa, Sc, Sd;
} mp_curve_t;

static bool mp_path(const mp_request_t *req, mp_path_t *p)
{
    uint32_t best = 0;
    p->dom = 0;
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        uint32_t n = (uint32_t)(req->steps[i] < 0 ? -(int64_t)req->steps[i] : req->steps[i]);
        if (n > best) {
            best = n;
            p->dom = i;
        }
    }
    p->n = best;
    if (best == 0U) {
        return false;
    }
    p->vmax = INFINITY;
    p->accel = INFINITY;
    p->vfloor = INFINITY;
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        uint32_t n = (uint32_t)(req->steps[i] < 0 ? -(int64_t)req->steps[i] : req->steps[i]);
        p->r[i] = (float)n / (float)best;
        if (n == 0U) {
            continue;
        }
        /* O eixo i anda r_i vezes o dominante: seus limites valem 1/r_i no dominante */
        float vmax = (req->vmax[i] > 0.0f) ? req->vmax[i] : 1.0f;
        float accel = (req->accel[i] > 0.0f) ? req->accel[i] : 1.0f;
        float vfloor = (req->v_floor[i] > 0.0f) ? req->v_floor[i] : 1.0f;
        p->vmax = fminf(p->vmax, vmax / p->r[i]);
        p->accel = fminf(p->accel, accel / p->r[i]);
        p->vfloor = fminf(p->vfloor, vfloor / p->r[i]);
    }
    if (p->vfloor > p->vmax) {
        p->vfloor = p->vmax;
    }
    return true;
}

float mp_max_rate(const mp_request_t *req)
{
    mp_path_t p;
    return mp_path(req, &p) ? p.vmax / (float)p.n : 0.0f;
}

float mp_stoppable_entry_rate(const mp_request_t *req)
{
    mp_path_t p;
    if (!mp_path(req, &p)) {
        return 0.0f;
    }
    /* Distancia da rampa S-curve va->vb: S = 0,75 * (va^2 - vb^2) / A */
    float v = sqrtf(p.vfloor * p.vfloor + 4.0f * p.accel * (float)p.n / 3.0f);
    return fminf(v, p.vmax) / (float)p.n;
}

static float ramp_distance(float va, float vb, float A)
{
    return 0.75f * fabsf(vb * vb - va * va) / A;
}

static void curve_finish(mp_curve_t *c, float n)
{
    c->Sa = (c->vc > c->v0) ? ramp_distance(c->v0, c->vc, c->A) : 0.0f;
    c->Sd = (c->vc > c->v1) ? ramp_distance(c->vc, c->v1, c->A) : 0.0f;
    c->Sc = n - c->Sa - c->Sd;
    if (c->Sc < 0.0f) {
        c->Sc = 0.0f;
    }
    c->Ta = (c->vc > c->v0) ? 1.5f * (c->vc - c->v0) / c->A : 0.0f;
    c->Td = (c->vc > c->v1) ? 1.5f * (c->vc - c->v1) / c->A : 0.0f;
    c->Tc = c->Sc / c->vc;
}

/* Resolve vc (e, se preciso, v1) para caber em n passos. */
static void curve_solve(mp_curve_t *c, float n)
{
    if (c->v0 > c->vc) c->v0 = c->vc;
    if (c->v1 > c->vc) c->v1 = c->vc;
    if (ramp_distance(c->v0, c->vc, c->A) + ramp_distance(c->vc, c->v1, c->A) > n) {
        /* Perfil triangular: 1,5/(2A) * (2vc^2 - v0^2 - v1^2) = n */
        float vc2 = (4.0f * c->A * n / 3.0f + c->v0 * c->v0 + c->v1 * c->v1) * 0.5f;
        c->vc = sqrtf(vc2);
        if (c->vc < c->v0 || c->vc < c->v1) {
            if (c->v0 >= c->v1) {
                /* Nao da para desacelerar ate v1: sai mais rapido (a juncao seguinte se adapta) */
                float v1_2 = c->v0 * c->v0 - 4.0f * c->A * n / 3.0f;
                c->v1 = (v1_2 > c->v1 * c->v1) ? sqrtf(v1_2) : c->v1;
                c->vc = c->v0;
            } else {
                /* Nao da para acelerar ate v1: sai na velocidade que alcancar */
                c->v1 = sqrtf(c->v0 * c->v0 + 4.0f * c->A * n / 3.0f);
                c->vc = c->v1;
            }
        }
    }
    curve_finish(c, n);
}

/* Posicao local na rampa de aceleracao (t em [0, Ta]) e velocidade. */
static float acc_pos(const mp_curve_t *c, float t, float *v)
{
    float u = t / c->Ta;
    float dv = c->vc - c->v0;
    *v = c->v0 + dv * u * u * (3.0f - 2.0f * u);
    return c->v0 * t + dv * c->Ta * (u * u * u - 0.5f * u * u * u * u);
}

/* Posicao local na rampa de desaceleracao (tau em [0, Td]). */
static float dec_pos(const mp_curve_t *c, float tau, float *v)
{
    float w = tau / c->Td;
    float dv = c->vc - c->v1;
    *v = c->vc - dv * w * w * (3.0f - 2.0f * w);
    return c->vc * tau - dv * c->Td * (w * w * w - 0.5f * w * w * w * w);
}

/* Inverte s(t) numa rampa (funcao crescente em [0, T]) por Newton com salvaguarda de bissecao. */
static float invert_ramp(const mp_curve_t *c, bool accel_ramp, float s, float guess)
{
    float T = accel_ramp ? c->Ta : c->Td;
    float lo = 0.0f, hi = T;
    float t = (guess > 0.0f && guess < T) ? guess : 0.5f * T;
    for (int it = 0; it < 30; ++it) {
        float v;
        float f = (accel_ramp ? acc_pos(c, t, &v) : dec_pos(c, t, &v)) - s;
        if (f > 0.0f) {
            hi = t;
        } else {
            lo = t;
        }
        if (fabsf(f) < 1e-5f || (hi - lo) < 1e-9f) {
            break;
        }
        float next = (v > 1e-6f) ? t - f / v : 0.5f * (lo + hi);
        t = (next > lo && next < hi) ? next : 0.5f * (lo + hi);
    }
    return t;
}

/* Tempo absoluto do passo em posicao s (so usado na cabeca, onde t <= Ta + pouco). */
static float head_time(const mp_curve_t *c, float s, float *guess)
{
    if (s <= c->Sa && c->Ta > 0.0f) {
        *guess = invert_ramp(c, true, s, *guess);
        return *guess;
    }
    if (s <= c->Sa + c->Sc) {
        return c->Ta + (s - c->Sa) / c->vc;
    }
    float tau = (c->Td > 0.0f) ? invert_ramp(c, false, s - c->Sa - c->Sc, 0.0f) : 0.0f;
    return c->Ta + c->Tc + tau;
}

/* Tempo relativo ao inicio da desaceleracao (evita somar tempos grandes em float). */
static float tail_time(const mp_curve_t *c, float s, float *guess)
{
    float s_dec = s - c->Sa - c->Sc;
    if (s_dec >= 0.0f) {
        if (c->Td <= 0.0f) {
            return s_dec / c->vc;
        }
        *guess = invert_ramp(c, false, s_dec, *guess);
        return *guess;
    }
    return s_dec / c->vc; /* ainda no cruzeiro: negativo */
}

static uint32_t to_q8(float dt, uint32_t tick_hz)
{
    double q = (double)dt * (double)tick_hz * 256.0;
    const double qmin = (double)MP_MIN_PERIOD_TICKS * 256.0;
    if (!(q >= qmin)) {
        q = qmin;
    }
    if (q > 4294967295.0) {
        q = 4294967295.0;
    }
    return (uint32_t)(q + 0.5);
}

static size_t pool_needed(const mp_path_t *p, const mp_curve_t *c)
{
    size_t words = 0;
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        if (p->r[i] > 0.0f) {
            words += (size_t)ceilf(p->r[i] * (c->Sa + c->Sd)) + 4U;
        }
    }
    return words;
}

mp_result_t mp_plan_move(const mp_request_t *req, uint32_t *pool, size_t pool_words,
                         uint32_t tick_hz, mp_plan_t *out)
{
    if (req == NULL || out == NULL || pool == NULL || tick_hz == 0U) {
        return MP_ERR_ARG;
    }
    memset(out, 0, sizeof(*out));
    mp_path_t p;
    if (!mp_path(req, &p)) {
        return MP_ERR_EMPTY;
    }
    const float n = (float)p.n;

    mp_curve_t c = {0};
    c.A = p.accel;
    c.vc = p.vmax;
    c.v0 = (req->rate_entry > 0.0f) ? fmaxf(req->rate_entry * n, p.vfloor) : p.vfloor;
    c.v1 = (req->rate_exit > 0.0f) ? fmaxf(req->rate_exit * n, p.vfloor) : p.vfloor;
    curve_solve(&c, n);

    /* Tabelas das rampas precisam caber no pool: reduz o cruzeiro (bissecao) se preciso */
    if (pool_needed(&p, &c) > pool_words) {
        float lo = fmaxf(c.v0, c.v1), hi = c.vc;
        mp_curve_t test = c;
        test.vc = lo;
        curve_finish(&test, n);
        if (pool_needed(&p, &test) > pool_words) {
            return MP_ERR_POOL;
        }
        for (int it = 0; it < 24; ++it) {
            test.vc = 0.5f * (lo + hi);
            curve_finish(&test, n);
            if (pool_needed(&p, &test) > pool_words) {
                hi = test.vc;
            } else {
                lo = test.vc;
            }
        }
        c.vc = lo;
        curve_finish(&c, n);
    }

    const float T = c.Ta + c.Tc + c.Td;
    size_t used = 0;
    uint64_t max_active = 0;

    for (uint8_t i = 0; i < MP_AXES; ++i) {
        mp_axis_prog_t *ax = &out->axis[i];
        uint32_t ni = (uint32_t)(req->steps[i] < 0 ? -(int64_t)req->steps[i] : req->steps[i]);
        ax->total_steps = ni;
        ax->dir_positive = req->steps[i] >= 0;
        if (ni == 0U) {
            continue;
        }
        const float r = p.r[i];

        /* Cabeca: passos na rampa de aceleracao + o que cruza a fronteira */
        uint32_t in_accel = (uint32_t)floorf(r * c.Sa + 0.5f);
        uint32_t head_end = (in_accel + 1U < ni) ? in_accel + 1U : ni;
        uint32_t cruise_end = (uint32_t)floorf(r * (c.Sa + c.Sc) + 0.5f);
        if (cruise_end > ni) cruise_end = ni;
        uint32_t n_cruise = (cruise_end > head_end) ? cruise_end - head_end : 0U;
        uint32_t tail_start = head_end + n_cruise + 1U;
        uint32_t n_tail = (tail_start <= ni) ? ni - tail_start + 1U : 0U;

        uint32_t *head = pool + used;
        used += head_end;
        uint32_t *tail = pool + used;
        used += n_tail;
        if (used > pool_words) {
            return MP_ERR_POOL;
        }

        uint64_t sum_q8 = 0;
        float guess = 0.0f, t_prev = 0.0f;
        for (uint32_t k = 1; k <= head_end; ++k) {
            float t = head_time(&c, ((float)k - 0.5f) / r, &guess);
            head[k - 1U] = to_q8(t - t_prev, tick_hz);
            sum_q8 += head[k - 1U];
            t_prev = t;
        }
        ax->cruise_q8 = to_q8(1.0f / (r * c.vc), tick_hz);
        sum_q8 += (uint64_t)ax->cruise_q8 * n_cruise;

        if (n_tail > 0U) {
            float tguess = 0.0f;
            float tau_prev = tail_time(&c, ((float)(tail_start - 1U) - 0.5f) / r, &tguess);
            for (uint32_t k = tail_start; k <= ni; ++k) {
                float tau = tail_time(&c, ((float)k - 0.5f) / r, &tguess);
                tail[k - tail_start] = to_q8(tau - tau_prev, tick_hz);
                sum_q8 += tail[k - tail_start];
                tau_prev = tau;
            }
        }

        ax->head = head;
        ax->n_head = head_end;
        ax->n_cruise = n_cruise;
        ax->tail = tail;
        ax->n_tail = n_tail;
        ax->active_ticks = sum_q8 >> 8; /* o ISR acumula a fracao: emite exatamente floor(soma/256) */
        if (ax->active_ticks > max_active) {
            max_active = ax->active_ticks;
        }
    }

    /* Mesma duracao para os tres canais: os eixos seguem alinhados movimento apos movimento */
    uint64_t ideal = (uint64_t)((double)T * (double)tick_hz + 0.5);
    /* >= 2 ticks de espera final em todo eixo: o gerador nunca emite duracao 0 */
    uint64_t duration = (ideal > max_active + 2U) ? ideal : max_active + 2U;
    if (duration > 0xFFFFFFFFULL) {
        return MP_ERR_ARG;
    }
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        out->axis[i].post_delay_ticks = (uint32_t)(duration - out->axis[i].active_ticks);
    }

    out->dom = p.dom;
    out->dom_steps = p.n;
    out->v_entry = c.v0;
    out->v_cruise = c.vc;
    out->v_exit = c.v1;
    out->rate_entry = c.v0 / n;
    out->rate_exit = c.v1 / n;
    out->accel_dom = c.A;
    out->duration_s = (float)duration / (float)tick_hz;
    out->duration_ticks = (uint32_t)duration;
    out->pool_used = used;
    return MP_OK;
}

bool mp_junction(const mp_request_t *a, const mp_request_t *b, const float jerk[MP_AXES],
                 float *rate_exit_a, float *rate_entry_b)
{
    mp_path_t pa, pb;
    if (!mp_path(a, &pa) || !mp_path(b, &pb)) {
        return false;
    }
    /* Proporcao entre as taxas que melhor casa as velocidades de cada eixo (minimos quadrados) */
    double ab = 0.0, bb = 0.0;
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        ab += (double)a->steps[i] * (double)b->steps[i];
        bb += (double)b->steps[i] * (double)b->steps[i];
    }
    if (bb <= 0.0 || ab <= 0.0) {
        return false; /* sentido oposto no conjunto: precisa parar */
    }
    float cratio = (float)(ab / bb);

    /* Salto por eixo: |a_i * x - b_i * c * x| <= jerk_i */
    float x = mp_max_rate(a);
    x = fminf(x, mp_max_rate(b) / cratio);
    x = fminf(x, mp_stoppable_entry_rate(b) / cratio);
    for (uint8_t i = 0; i < MP_AXES; ++i) {
        float jump = fabsf((float)a->steps[i] - (float)b->steps[i] * cratio);
        if (jump > 1e-6f) {
            float j = (jerk[i] > 0.0f) ? jerk[i] : 0.0f;
            x = fminf(x, j / jump);
        }
    }
    float y = cratio * x;
    /* Abaixo da velocidade de partida nao compensa encadear: para e parte normalmente */
    if (!(x * (float)pa.n > pa.vfloor) || !(y * (float)pb.n > pb.vfloor)) {
        return false;
    }
    *rate_exit_a = x;
    *rate_entry_b = y;
    return true;
}
