#pragma once

/*
 * Planejador de perfis de movimento (C puro, sem dependencias do ESP-IDF: testavel no PC).
 *
 * Perfil S-curve no dominio do TEMPO: em cada rampa a velocidade segue
 * v(t) = va + (vb - va) * (3u^2 - 2u^3), u = t/T, com T = 1,5 * |vb - va| / A.
 * A aceleracao comeca e termina em zero (sem tranco) e seu pico e exatamente A.
 * (O perfil antigo aplicava o smoothstep sobre o indice do PASSO, o que dava pico
 * de ~1,5x a aceleracao configurada.)
 *
 * Varios eixos: o eixo com mais passos e o dominante; cada eixo i percorre
 * s_i(t) = r_i * s_dom(t), com r_i = N_i / N_dom — todos terminam juntos e as
 * trajetorias ficam coordenadas no tempo.
 *
 * Saida por eixo: periodos entre passos em ticks do RMT, em ponto fixo Q8 (1/256 tick),
 * so para as rampas (cabeca e cauda). O trecho de cruzeiro e um periodo constante.
 * O ISR so soma inteiros para gerar os simbolos.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MP_AXES 3
#define MP_PULSE_TICKS 3U          /* largura do pulso STEP (ticks do RMT) */
#define MP_MIN_PERIOD_TICKS (MP_PULSE_TICKS + 1U)

typedef struct {
    int32_t steps[MP_AXES];   /* passos com sinal por eixo */
    float vmax[MP_AXES];      /* velocidade maxima por eixo (passos/s) */
    float accel[MP_AXES];     /* aceleracao maxima por eixo (passos/s^2) */
    float v_floor[MP_AXES];   /* velocidade de partida/parada por eixo (passos/s) */
    float rate_entry;         /* velocidade de entrada como taxa de progresso (1/s); 0 = partir do repouso */
    float rate_exit;          /* velocidade de saida desejada (1/s); 0 = parar */
} mp_request_t;

typedef struct {
    uint32_t total_steps;
    bool dir_positive;
    uint32_t n_head;          /* periodos da rampa inicial (inclui o passo que cruza a fronteira) */
    uint32_t n_cruise;
    uint32_t cruise_q8;       /* periodo constante do cruzeiro (Q8) */
    uint32_t n_tail;
    const uint32_t *head;     /* n_head periodos Q8 */
    const uint32_t *tail;     /* n_tail periodos Q8 */
    uint32_t post_delay_ticks;/* espera apos o ultimo passo: todos os eixos terminam no mesmo tick */
    uint64_t active_ticks;    /* soma dos periodos (floor(sum Q8 / 256)) */
} mp_axis_prog_t;

typedef struct {
    mp_axis_prog_t axis[MP_AXES];
    uint8_t dom;              /* eixo dominante */
    uint32_t dom_steps;
    float v_entry, v_cruise, v_exit; /* velocidades do eixo dominante (passos/s) efetivas */
    float rate_entry, rate_exit;     /* mesmas velocidades como taxa de progresso (1/s) */
    float accel_dom;          /* aceleracao efetiva do dominante (passos/s^2) */
    float duration_s;
    uint32_t duration_ticks;  /* duracao total, igual para os tres canais */
    size_t pool_used;         /* palavras do pool usadas pelas tabelas */
} mp_plan_t;

typedef enum {
    MP_OK = 0,
    MP_ERR_EMPTY = -1,        /* nenhum passo */
    MP_ERR_POOL = -2,         /* rampas nao cabem no pool nem reduzindo a velocidade */
    MP_ERR_ARG = -3,
} mp_result_t;

/* Planeja um movimento. `pool` recebe as tabelas das rampas (reutilizado entre movimentos). */
mp_result_t mp_plan_move(const mp_request_t *req, uint32_t *pool, size_t pool_words,
                         uint32_t tick_hz, mp_plan_t *out);

/* Taxa de progresso maxima (1/s) de um movimento — cruzeiro limitado pelo eixo mais restrito. */
float mp_max_rate(const mp_request_t *req);

/* Maior taxa de entrada (1/s) da qual o movimento ainda consegue parar ate o fim. */
float mp_stoppable_entry_rate(const mp_request_t *req);

/* ------------------------------------------------------------------------------------ */
/* Seguidor de alvo em tempo real (jog e streaming absoluto U): um eixo, periodo fixo dt.  */
/* ------------------------------------------------------------------------------------ */

#define MP_TRACK_MAX_SMOOTH 16U

typedef struct {
    float vmax;        /* passos/s */
    float accel;       /* passos/s^2 */
    uint8_t smooth_n;  /* periodos da media movel (jerk <= 2*accel / (smooth_n * dt)); 1 = sem */
} mp_track_limits_t;

typedef struct {
    float pos_in, vel_in;             /* seguidor interno (aceleracao limitada) */
    float hist[MP_TRACK_MAX_SMOOTH];  /* deslocamentos dos ultimos periodos */
    uint8_t idx;
    float pos;                        /* posicao comandada suavizada (passos, com fracao) */
    float vel;                        /* passos/s */
    float acc;                        /* passos/s^2 */
} mp_track_state_t;

float mp_track_brake_speed(float dist, float accel);
/* Periodos da media movel que dao o jerk pedido (1..MP_TRACK_MAX_SMOOTH). */
uint8_t mp_track_smooth_periods(float accel, float jerk, float dt);
void mp_track_reset(mp_track_state_t *s, float pos);

/*
 * Avanca um periodo `dt`. `target` e o alvo no inicio do periodo e `target_vel` sua
 * velocidade (feedforward): com ela o seguidor acompanha um alvo em movimento sem o
 * atraso v/2a de um seguidor so de posicao. Seguidor interno com aceleracao limitada
 * que nunca cruza o alvo + media movel de smooth_n periodos (curva S, jerk limitado).
 * A media atrasa (smooth_n - 1)/2 periodos: compense avaliando o alvo a frente.
 */
void mp_track_step(mp_track_state_t *s, const mp_track_limits_t *lim, float target, float target_vel, float dt);
/* true quando parado e sem deslocamento pendente na media. */
bool mp_track_idle(const mp_track_state_t *s);

/*
 * Feedforward de um alvo recebido em amostras (frames U do TouchDesigner, taxa qualquer):
 * estima a velocidade pela diferenca entre amostras e extrapola o alvo entre elas, ate
 * 1,25 intervalo (folga para o jitter de USB/CAN). Sem amostra nova, o alvo para ali.
 * Taxa variavel e suportada: o intervalo medio se adapta e lacunas (> 250 ms ou frame
 * perdido) zeram a velocidade em vez de gerar um salto.
 */
typedef struct {
    float target;        /* ultima amostra */
    float vel;           /* unidades/s */
    float interval_s;    /* intervalo medio entre amostras (0 = desconhecido) */
    int64_t t_us;        /* instante da ultima amostra */
    bool valid;
} mp_track_feed_t;

void mp_track_feed_reset(mp_track_feed_t *f);
void mp_track_feed_update(mp_track_feed_t *f, float target, int64_t t_us);
/* Alvo extrapolado para t_us + lead_s (lead compensa o atraso do pipeline/suavizacao). */
void mp_track_feed_eval(const mp_track_feed_t *f, int64_t t_us, float lead_s, float *target, float *vel);

/* ------------------------------------------------------------------------------------ */
/* Gerador de simbolos (ISR): so aritmetica inteira — float e proibido em ISR no Xtensa.   */
/* ------------------------------------------------------------------------------------ */

#define MP_SYM_MAX_DUR 32767U
/* Mesmo layout de rmt_symbol_word_t: duration0:15 | level0:1 | duration1:15 | level1:1 */
#define MP_SYM(l0, d0, l1, d1) \
    ((uint32_t)(d0) | ((uint32_t)(l0) << 15) | ((uint32_t)(d1) << 16) | ((uint32_t)(l1) << 31))

typedef struct {
    const mp_axis_prog_t *prog;
    uint32_t k;          /* passos ja iniciados */
    uint32_t frac;       /* fracao Q8 acumulada (o tick medio fica exato) */
    uint32_t low_left;   /* ticks em nivel baixo antes do pulso pendente */
    uint32_t post_left;  /* espera final restante */
    bool pulse_pending;
} mp_gen_t;

static inline __attribute__((always_inline)) void mp_gen_reset(mp_gen_t *g, const mp_axis_prog_t *prog)
{
    g->prog = prog;
    g->k = 0;
    g->frac = 0;
    g->low_left = 0;
    g->post_left = prog->post_delay_ticks;
    g->pulse_pending = false;
}

static inline __attribute__((always_inline)) uint32_t mp_gen_period_q8(const mp_axis_prog_t *p, uint32_t k)
{
    if (k < p->n_head) {
        return p->head[k];
    }
    k -= p->n_head;
    if (k < p->n_cruise) {
        return p->cruise_q8;
    }
    return p->tail[k - p->n_cruise];
}

/*
 * Escreve ate `space` simbolos. Cada passo = um simbolo {baixo: periodo - pulso, alto: pulso};
 * periodos longos ganham simbolos de preenchimento em nivel baixo antes. Depois do ultimo
 * passo emite a espera final. Retorna quantos simbolos escreveu; *done indica o fim.
 */
static inline __attribute__((always_inline)) size_t mp_gen_fill(mp_gen_t *g, uint32_t *words, size_t space, bool *done)
{
    size_t n = 0;
    while (n < space) {
        if (!g->pulse_pending) {
            if (g->k < g->prog->total_steps) {
                uint32_t acc = mp_gen_period_q8(g->prog, g->k) + g->frac;
                uint32_t ticks = acc >> 8;
                g->frac = acc & 0xFFU;
                g->k++;
                g->low_left = (ticks > MP_PULSE_TICKS) ? ticks - MP_PULSE_TICKS : 1U;
                g->pulse_pending = true;
            } else if (g->post_left > 0U) {
                uint32_t chunk = (g->post_left < 2U * MP_SYM_MAX_DUR) ? g->post_left : 2U * MP_SYM_MAX_DUR;
                if (g->post_left - chunk == 1U) {
                    chunk -= 1U; /* nunca deixar 1 tick sozinho (duracao 0 = fim da transmissao) */
                }
                uint32_t d0 = chunk / 2U;
                words[n++] = MP_SYM(0, d0, 0, chunk - d0);
                g->post_left -= chunk;
                continue;
            } else {
                *done = true;
                break;
            }
        }
        if (g->low_left > MP_SYM_MAX_DUR) {
            uint32_t chunk = g->low_left - 1U; /* deixa >= 1 tick para o simbolo do pulso */
            if (chunk > 2U * MP_SYM_MAX_DUR) chunk = 2U * MP_SYM_MAX_DUR;
            uint32_t d0 = chunk / 2U;
            words[n++] = MP_SYM(0, d0, 0, chunk - d0);
            g->low_left -= chunk;
        } else {
            words[n++] = MP_SYM(0, g->low_left, 1, MP_PULSE_TICKS);
            g->low_left = 0;
            g->pulse_pending = false;
        }
    }
    if (n < space && !g->pulse_pending && g->k >= g->prog->total_steps && g->post_left == 0U) {
        *done = true;
    }
    return n;
}

/*
 * Juncao entre dois movimentos consecutivos (lookahead, como o "jerk" das impressoras 3D).
 * Escolhe as taxas de saida de `a` e de entrada de `b` de modo que o salto de velocidade
 * de cada eixo fique <= jerk[i] (passos/s). Retorna false se os movimentos devem parar
 * entre si (sentido oposto, salto grande demais ou velocidade abaixo da de partida).
 */
bool mp_junction(const mp_request_t *a, const mp_request_t *b, const float jerk[MP_AXES],
                 float *rate_exit_a, float *rate_entry_b);
