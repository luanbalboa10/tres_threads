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
K_THREAD_STACK_DEFINE(thread_stack, STACK_SIZE);
struct k_thread thread_data; /* Estrutura que contém os dados da thread, como contexto de execução, estado da thread, etc. */

K_THREAD_STACK_DEFINE(gpio_thread_stack, STACK_SIZE);
struct k_thread gpio_thread_data;

K_THREAD_STACK_DEFINE(sync_thread_stack, STACK_SIZE);
struct k_thread sync_thread_data;

K_MUTEX_DEFINE(my_mutex); /* define um mutex chamado my_mutex */

/* define dois semáforos, o segundo e terceiro parâmetro diz respeito aos limites de cada semáforo */
K_SEM_DEFINE(periodic_sem, 0, 1);
K_SEM_DEFINE(event_sem, 0, 1);

/* primeira thread */
void periodic_thread(void)
{
    while (1) {
        k_sleep(K_MSEC(1000));
        printk("Periodic thread running\n");
        k_mutex_lock(&my_mutex, K_FOREVER); /* trava o mutex - K_FOREVER: espera infinitamente até que o mutex esteja liberado */
        /* se fosse K_NO_WAIT: não espera o mutex, se ele não tava disponível naquele momento ele desiste de tentar pegar */
        k_sem_give(&periodic_sem); /* incrementa o valor do semáforo */
        k_mutex_unlock(&my_mutex); /* libera o mutex para que outros processos possam executar */
    }
}

/* função de callback da interrupção da thread 2 */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    printk("Button pressed!\n");
    k_mutex_lock(&my_mutex, K_FOREVER);
    k_sem_give(&event_sem);
    k_mutex_unlock(&my_mutex);
}

/* segunda thread */
void gpio_thread(const struct device *gpio_dev)
{
    static struct gpio_callback button_cb_data;

    gpio_pin_configure(gpio_dev, PIN, GPIO_INPUT | FLAGS);
    gpio_init_callback(&button_cb_data, button_pressed, BIT(PIN));
    gpio_add_callback(gpio_dev, &button_cb_data);
    gpio_pin_interrupt_configure(gpio_dev, PIN, GPIO_INT_EDGE_TO_ACTIVE);

    while (1) {
        k_sleep(K_FOREVER);
    }
}

/* terceira thread */
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
void create_threads(const struct device *gpio_dev)
{
    k_thread_create(&thread_data, thread_stack,
                    STACK_SIZE, (k_thread_entry_t)periodic_thread, 
                    NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_create(&gpio_thread_data, gpio_thread_stack, 
                    STACK_SIZE, (k_thread_entry_t)gpio_thread, 
                    gpio_dev, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
    k_thread_create(&sync_thread_data, sync_thread_stack, 
                    STACK_SIZE, (k_thread_entry_t)sync_thread, 
                    NULL, NULL, NULL, THREAD_PRIORITY, 0, K_NO_WAIT);
}

void main(void)
{
    const struct device *gpio_dev = device_get_binding(GPIO_PORT);
    __ASSERT(gpio_dev != NULL, "Failed to bind to GPIO device");

    create_threads(gpio_dev);

    while (1) {
        k_sleep(K_SECONDS(10));
        printk("Main loop running\n");
    }
}
