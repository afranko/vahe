#include "queue.h"
#include <string.h>
#include "platform.h"

#define BUF_PTR(ppool,i) ((struct buffer_t*)((uint8_t*)((ppool)->pool_bufs) + i*(sizeof(struct buffer_t) + (ppool)->buf_size)))

__DEFINE_IRQ_FLAG;

void *buffer_alloc(struct buffer_pool_t* pool, uint8_t size)
{
    if(pool->buf_size < size)
        return NULL;

    __DISABLE_IRQ();

    int i;
    for(i=0; i < pool->pool_size; i++)
    {
        if(BUF_PTR(pool,i)->buf_free)
        {
            BUF_PTR(pool,i)->buf_free = 0;
            BUF_PTR(pool,i)->buf_length = size;

            __ENABLE_IRQ();
            return BUF_PTR(pool,i)->buf;
        }
    }

    __ENABLE_IRQ();
    return NULL;
}



void buffer_pool_init(struct buffer_pool_t *pool)
{
    int i;
    for(i = 0; i < pool->pool_size; i++)
    {
        BUF_PTR(pool,i)->buf_free = 1;
    }
}

void buffer_free(struct buffer_pool_t *pool, void *ptr)
{
    __DISABLE_IRQ();
    ((struct buffer_t*)(((uint8_t*)ptr) - sizeof(struct buffer_t)))->buf_free = 1;
    __ENABLE_IRQ();
}

BUFFER_LTYPE buffer_length(struct buffer_pool_t *pool, void *ptr)
{
    return ((struct buffer_t*)(((uint8_t*)ptr) - sizeof(struct buffer_t)))->buf_length;
}


int queue_push(struct queue_t *q, void *p)
{
    __DISABLE_IRQ();
    if(q->size >= q->capacity)
    {
        __ENABLE_IRQ();
        return -1;
    }

    q->p[q->wptr] = p;
    q->wptr = (q->wptr + 1) % q->capacity;
    q->size++;

    __ENABLE_IRQ();
    return 0;
}


void *queue_pop(struct queue_t *q)
{
    __DISABLE_IRQ();
    if(q->size == 0)
    {
        __ENABLE_IRQ();
        return NULL;
    }

    void* ret = q->p[q->rptr];
    q->size--;
    q->rptr = (q->rptr + 1) % q->capacity;

    __ENABLE_IRQ();
    return ret;
}
