#ifndef __QUEUE_H_
#define __QUEUE_H_

#include <stdint.h>

#define BUFFER_LTYPE  uint8_t

struct buffer_t
{
    uint8_t                 buf_free;
    BUFFER_LTYPE            buf_length;
    uint16_t                padding;        // to be 32 bit aligned
    uint8_t                 buf[0];
};

struct buffer_pool_t
{
    uint8_t                        pool_size;
    BUFFER_LTYPE                   buf_size;
    struct buffer_t*               pool_bufs;
};

#define BUFFER_POOL_CREATE(name,pool_size,buf_size)    \
    uint8_t                 name##_mem[(pool_size)*(sizeof(struct buffer_t) + (buf_size))]; \
    struct buffer_pool_t    name = { (pool_size), (buf_size), (struct buffer_t*)name##_mem }
#define BUFFER_POOL_EXTERN(name)    extern struct buffer_pool_t name

void    buffer_pool_init(struct buffer_pool_t* pool);
void*   buffer_alloc(struct buffer_pool_t* pool, BUFFER_LTYPE size);
void    buffer_free(struct buffer_pool_t* pool, void* ptr);
BUFFER_LTYPE    buffer_length(struct buffer_pool_t* pool, void* ptr);

struct queue_t
{
    uint8_t     capacity;
    uint8_t     size;
    uint8_t     rptr;
    uint8_t     wptr;

    void**      p;
};

#define QUEUE_CREATE(name,capacity) \
    uint8_t         name##_mem[(capacity) * sizeof(void*)]; \
    struct queue_t  name = { (capacity), 0, 0, 0, (void**)name##_mem }
#define QUEUE_EXTERN(name)  extern struct queue_t name

int    queue_push(struct queue_t* q, void* p);
void*  queue_pop(struct queue_t* q);


#endif
