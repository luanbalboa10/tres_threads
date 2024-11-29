/*
 * -------------------------------------------------------------------
 * Projeto: Três threads
 *
 * Descrição: Implementação de três threads. Com:
 * - uma função ativada periodicamente
 * - uma função ativada por evento (GPIO)
 * - uma função ativada quando as outras duas forem ativadas.
 *
 * Para esse projeto utilizamos somente uma thread para fazer a ativação da função de uma função por meio das outras duas. Utilizamos também dois semáforos e um mutex. 
 *
 * Desenvolvedores:
 * João Xavier 
 * Luan Cotini
 * 
 * Data de criação: 29/11/2024
 *
 * -------------------------------------------------------------------
*/
/* Inclui os cabeçalhos do Zephyr para trabalhar com GPIO, funções de impressão */
#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <sys/__assert.h>
#include <logging/log.h>

#define GPIO_PORT DT_ALIAS_SW0_GPIOS_CONTROLLER
#define PIN DT_ALIAS_SW0_GPIOS_PIN
#define FLAGS DT_ALIAS_SW0_GPIOS_FLAGS

LOG_MODULE_REGISTER(main);

#define STACK_SIZE 1024 /* tamanho da pilha */ 
#define THREAD_PRIORITY 7 /* define a prioridade das threads */

/* cria e define a pilha de cada thread */
K_THREAD_STACK_DEFINE(sync_thread_stack, STACK_SIZE);
struct k_thread sync_thread_data; /* Estrutura que contém os dados da thread, como contexto de execução, estado da thread, etc. */

K_MUTEX_DEFINE(my_mutex); /* define um mutex chamado my_mutex */

/* define dois semáforos, o segundo e terceiro parâmetro diz respeito aos limites de cada semáforo */
K_SEM_DEFINE(periodic_sem, 0, 1);
K_SEM_DEFINE(event_sem, 0, 1);

/* Função de callback do temporizador */
void timer_expiry_function(struct k_timer *timer_id)
{    
        printk("Função periódica executando! \n");
        k_mutex_lock(&my_mutex, K_FOREVER); /* espera infinitamente até que o mutex esteja liberado */
        /* se fosse K_NO_WAIT: não espera o mutex, se ele não tava disponível naquele momento ele desiste de tentar pegar */
        k_sem_give(&periodic_sem); /* incrementa o valor do semáforo */
        k_mutex_unlock(&my_mutex); /* libera o mutex para que outros processos possam executar */
}

/* Define o temporizador */
K_TIMER_DEFINE(periodic_timer, timer_expiry_function, NULL);

void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Botão pressionado!\n");
    k_mutex_lock(&my_mutex, K_FOREVER); 
    k_sem_give(&event_sem);
    k_mutex_unlock(&my_mutex);
}

/* função de callback do GPIO */
void setup_gpio_thread(const struct device *gpio_dev)
{
    /* declaração da estrutura estática de callback gpio_callback button_cb_data */
    static struct gpio_callback button_cb_data; 

    gpio_pin_configure(gpio_dev, PIN, GPIO_INPUT | FLAGS); /* configura um pino */
    gpio_init_callback(&button_cb_data, button_pressed, BIT(PIN)); /* inicializa a estrutura       do callback */
    gpio_add_callback(gpio_dev, &button_cb_data); /* registra a estrutura do callback */
    gpio_pin_interrupt_configure(gpio_dev, PIN, GPIO_INT_EDGE_TO_ACTIVE); /* configura a           interrupção em um pino */
}

/* thread pra sincronismo */
void sync_thread(void)
{
    while (1) {
        k_sem_take(&periodic_sem, K_FOREVER);
        printk("Sync thread running after periodic\n");
        k_sem_take(&event_sem, K_FOREVER);
        printk("Sync thread running after event\n");
    }
}

/* cria as threads */
void create_threads()
{
    k_thread_create(&sync_thread_data, sync_thread_stack, 
                    STACK_SIZE, (k_thread_entry_t)sync_thread, 
                    NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
}

void main(void)
{
    const struct device *gpio_dev = device_get_binding(GPIO_PORT);
    __ASSERT(gpio_dev != NULL, "Failed to bind to GPIO device");

    setup_gpio_callback(gpio_dev); /* configura o setup de GPIO */
    create_threads(); /* cria a thread */

    /* inicia o temporizador para expirar a cada segundo (1000 ms) */
    k_timer_start(&periodic_timer, K_SECONDS(1), K_SECONDS(1));
    while (1) {
        k_sleep(K_SECONDS(10));
        printk("Main loop running\n");
    }
}
