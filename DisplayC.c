#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "inc/ssd1306.h"
#include "inc/font.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "matriz_led.pio.h"

#define I2C_PORT i2c1
#define I2C_SDA 14
#define I2C_SCL 15
#define endereco 0x3C
#define NUM_PIXELS 25      // número de leds na matriz
#define LED_PIN 7          // pino de saída do led
#define LED_VERDE 11       // pino de saída do led verde
#define LED_AZUL 12        // pino de saída do led verde
#define BOTAO_A 5          // Pino do botão A
#define BOTAO_B 6          // Pino do botão B
#define DEBOUNCE_DELAY 300 // Tempo de debounce em milissegundos

volatile int NUMERO_APARENTE = 0; // Variável global para armazenar o número exibido (0 a 9)
volatile uint32_t last_interrupt_time = 0;

PIO pio;
uint sm;
float r = 1.0, g = 1.0, b = 1.0;

void gpio_callback(uint gpio, uint32_t events)
{
  static uint32_t last_gpio = 0;

  uint32_t current_time = to_ms_since_boot(get_absolute_time());

  // Verifica se é o mesmo botão pressionado e se está dentro do tempo de debounce
  if (gpio == last_gpio && (current_time - last_interrupt_time < DEBOUNCE_DELAY))
    return;

  last_interrupt_time = current_time;
  last_gpio = gpio;

  if (gpio == BOTAO_A)
  {
    if (gpio_get(LED_VERDE) == 0)
    {                         // Verifique o estado atual do LED
      gpio_put(LED_VERDE, 1); // Acende o LED se ele estiver apagado
      printf("Led verde acendeu\n");
    }
    else
    {
      gpio_put(LED_VERDE, 0); // Apaga o LED se ele estiver aceso
      printf("Led verde apagou\n");
    }
  }
  else if (gpio == BOTAO_B)
  {
    if (gpio_get(LED_AZUL) == 0)
    {                        // Verifique o estado atual do LED
      gpio_put(LED_AZUL, 1); // Acende o LED se ele estiver apagado
      printf("Led azul acendeu\n");
    }
    else
    {
      gpio_put(LED_AZUL, 0); // Apaga o LED se ele estiver aceso
      printf("Led azul apagou\n");
    }
  }
}

// MATRIZ DE LEDS
// rotina para definição da intensidade de cores do led
uint matrix_rgb(float r, float g, float b)
{
  unsigned char R, G, B;
  R = r * 255;
  G = g * 255;
  B = b * 255;
  return (G << 24) | (R << 16) | (B << 8);
}

// Função para converter a posição do matriz para uma posição do vetor.
int getIndex(int x, int y)
{
  // Se a linha for par (0, 2, 4), percorremos da esquerda para a direita.
  // Se a linha for ímpar (1, 3), percorremos da direita para a esquerda.
  if (y % 2 == 0)
  {
    return 24 - (y * 5 + x); // Linha par (esquerda para direita).
  }
  else
  {
    return 24 - (y * 5 + (4 - x)); // Linha ímpar (direita para esquerda).
  }
}

void desenho_pio(double *desenho)
{
  for (int16_t i = 0; i < NUM_PIXELS; i++)
  {
    uint32_t valor_led = matrix_rgb(desenho[i] * r, desenho[i] * g, desenho[i] * b);
    pio_sm_put_blocking(pio, sm, valor_led);
  }
}

double numero[10][25] = {
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 0.0, 0.0,
     0.0, 0.0, 1.0, 0.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 0.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
    {0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 0.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0,
     0.0, 1.0, 0.0, 1.0, 0.0,
     0.0, 1.0, 1.0, 1.0, 0.0},
};

int main()
{
  pio = pio0;
  bool frequenciaClock;

  frequenciaClock = set_sys_clock_khz(128000, false);
  stdio_init_all();

  gpio_init(LED_VERDE);
  gpio_set_dir(LED_VERDE, GPIO_OUT);

  gpio_init(LED_AZUL);
  gpio_set_dir(LED_AZUL, GPIO_OUT);

  gpio_init(BOTAO_A);
  gpio_set_dir(BOTAO_A, GPIO_IN);
  gpio_pull_up(BOTAO_A);

  gpio_init(BOTAO_B);
  gpio_set_dir(BOTAO_B, GPIO_IN);
  gpio_pull_up(BOTAO_B);

  gpio_set_irq_enabled_with_callback(BOTAO_A, GPIO_IRQ_EDGE_FALL, true, gpio_callback);
  gpio_set_irq_enabled_with_callback(BOTAO_B, GPIO_IRQ_EDGE_FALL, true, gpio_callback);

  uint offset = pio_add_program(pio, &pio_matrix_program);
  sm = pio_claim_unused_sm(pio, true);
  pio_matrix_program_init(pio, sm, offset, LED_PIN);

  // I2C inicializada. Usando-o em400Khz.
  i2c_init(I2C_PORT, 400 * 1000);
  gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);                    // Define o pino da GPIO para I2C
  gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);                    // Define o pino da GPIO para I2C
  gpio_pull_up(I2C_SDA);                                        // Pull up no data line
  gpio_pull_up(I2C_SCL);                                        // Pull up no clock line
  ssd1306_t ssd;                                                // Inicializa a estrutura do display
  ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT); // Inicializa o display
  ssd1306_config(&ssd);                                         // Configura o display
  ssd1306_send_data(&ssd);                                      // Envia os dados para o display

  // Limpa o display. O display inicia com todos os pixels apagados.
  ssd1306_fill(&ssd, false);
  ssd1306_send_data(&ssd);

  bool cor = true;

  while (true)
  {
    if (gpio_get(BOTAO_A) == 0)
    {
      ssd1306_draw_string(&ssd, "Botao A pressionado", 8, 10); // Desenha uma string
      ssd1306_draw_string(&ssd, "Led verde alterado", 8, 30);  // Desenha uma string
      ssd1306_send_data(&ssd);                                 // Atualiza o display
    }

    if (gpio_get(BOTAO_B) == 0)
    {
      ssd1306_draw_string(&ssd, "Botao B pressionado", 8, 10); // Desenha uma string
      ssd1306_draw_string(&ssd, "Led azul alterado", 8, 30);   // Desenha uma string
      ssd1306_send_data(&ssd);                                 // Atualiza o display
    }

    if (stdio_usb_connected())
    { // Certifica-se de que o USB está conectado
      char c;

      if (scanf("%c", &c) == 1)
      { // Lê caractere da entrada padrão
        printf("Recebido: '%c'\n", c);

        ssd1306_fill(&ssd, !cor);                     // Limpa o display
        ssd1306_rect(&ssd, 3, 3, 122, 58, cor, !cor); // Desenha um retângulo
        char buffer[2] = {c, '\0'};                   // Cria uma string com o caractere e o terminador nulo
        ssd1306_draw_string(&ssd, buffer, 8, 10);     // Desenha uma string
        ssd1306_send_data(&ssd);                      // Atualiza o display

        switch (c)
        {
        case '0':
          desenho_pio(numero[0]);
          printf("número 0!\n");
          break;
        case '1':
          desenho_pio(numero[1]);
          printf("número 1!\n");
          break;
        case '2':
          desenho_pio(numero[2]);
          printf("número 2!\n");
          break;
        case '3':
          desenho_pio(numero[3]);
          printf("número 3!\n");
          break;
        case '4':
          desenho_pio(numero[4]);
          printf("número 4!\n");
          break;
        case '5':
          desenho_pio(numero[5]);
          printf("número 5!\n");
          break;
        case '6':
          desenho_pio(numero[6]);
          printf("número 6!\n");
          break;
        case '7':
          desenho_pio(numero[7]);
          printf("número 7!\n");
          break;
        case '8':
          desenho_pio(numero[8]);
          printf("número 8!\n");
          break;
        case '9':
          desenho_pio(numero[9]);
          printf("número 9!\n");
          break;
        }
      }
    }
  }
}