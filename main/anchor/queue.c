#include "queue.h"
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_system.h"

//#include "platform.h"

#define BUF_PTR(ppool,i) ((struct buffer_t*)((uint8_t*)((ppool)->pool_bufs) + i*(sizeof(struct buffer_t) + (ppool)->buf_size)))

//__DEFINE_IRQ_FLAG;

SemaphoreHandle_t bufferMutex = NULL; //MUTEX for buffer

void *buffer_alloc(struct buffer_pool_t* pool, uint8_t size)
{
    if(pool->buf_size < size)
        return NULL;

    //__DISABLE_IRQ();

	if(xSemaphoreTake(bufferMutex, ( TickType_t ) 3000) == pdTRUE)
	{

	    int i;
	    for(i = 0; i < pool->pool_size; i++)
	    {
	        if(BUF_PTR(pool,i)->buf_free)
	        {
	            BUF_PTR(pool,i)->buf_free = 0;
	            BUF_PTR(pool,i)->buf_length = size;

	            //__ENABLE_IRQ();
				xSemaphoreGive(bufferMutex);
	            return BUF_PTR(pool,i)->buf;
	        }
	    }

	    //__ENABLE_IRQ();
		xSemaphoreGive(bufferMutex);
	}
    return NULL;
}

void buffer_pool_init(struct buffer_pool_t *pool)
{
    bufferMutex = xSemaphoreCreateMutex();
    if((void*)bufferMutex == NULL) {
    	ets_printf("MEMORY POOL SEMAFOR ERROR!\n");
    }
    int i;
    for(i = 0; i < pool->pool_size; i++)
    {
        BUF_PTR(pool,i)->buf_free = 1;
    }
}

void buffer_free(struct buffer_pool_t *pool, void *ptr)
{
    //__DISABLE_IRQ();
	if(xSemaphoreTake(bufferMutex, ( TickType_t ) portMAX_DELAY) == pdTRUE)	// TODO mást nem tudok itt csinálni, mert nincs hiba kezelés
	{
    	((struct buffer_t*)(((uint8_t*)ptr) - sizeof(struct buffer_t)))->buf_free = 1;
		xSemaphoreGive(bufferMutex);
	}
    //__ENABLE_IRQ();
}

BUFFER_LTYPE buffer_length(struct buffer_pool_t *pool, void *ptr)
{
    return ((struct buffer_t*)(((uint8_t*)ptr) - sizeof(struct buffer_t)))->buf_length;
}


int queue_push(struct queue_t *q, void *p)
{
    //__DISABLE_IRQ();
	if(xSemaphoreTake(bufferMutex, ( TickType_t ) 3000) == pdTRUE)	// TODO mást nem tudok itt csinálni, mert nincs hiba kezelés
	{
	    if(q->size >= q->capacity)
	    {
	        //__ENABLE_IRQ();
			xSemaphoreGive(bufferMutex);
	        return -1;
	    }

	    q->p[q->wptr] = p;
	    q->wptr = (q->wptr + 1) % q->capacity;
	    q->size++;

	    //__ENABLE_IRQ();
		xSemaphoreGive(bufferMutex);
	}
    return 0;
}


void *queue_pop(struct queue_t *q)
{
    //__DISABLE_IRQ();
	if(xSemaphoreTake(bufferMutex, ( TickType_t ) 3000) == pdTRUE)	// TODO mást nem tudok itt csinálni, mert nincs hiba kezelés
	{
	    if(q->size == 0)
	    {
	        //__ENABLE_IRQ();
			xSemaphoreGive(bufferMutex);
	        return NULL;
	    }

	    void* ret = q->p[q->rptr];
	    q->size--;
	    q->rptr = (q->rptr + 1) % q->capacity;

	    //__ENABLE_IRQ()
		xSemaphoreGive(bufferMutex);

        return ret;
	}
    return NULL;
}
