#include <stdint.h>

// Circular buffer for queues
struct T_queue {
  // max size just in case. A better implementation would dynamically
  // allocate this. But then you have to clean up
  uint64_t arr[QUEUE_SIZE]; 
  uint32_t virtual_size;
  volatile int head;
  volatile int tail;

  void set_size(uint32_t size) {
    assert(size <= QUEUE_SIZE);
    virtual_size = size;
  }

  void init(uint32_t size) {
    set_size(size);
    head = tail = 0;
  }

  bool is_full() {    
    return tail == ((head + 1) % virtual_size);
  }
  
  bool is_empty() {
    return (head == tail);
  }
  
  void enqueue(uint64_t v) {
    int index = head;
    arr[index] = v;
    head = ((index + 1) % virtual_size);
  }
  
  uint64_t dequeue() {
    int index = tail;
    uint64_t ret = arr[index];
    tail = ((index + 1) % virtual_size);
    return ret;
  }
  
  void do_enqueue(uint64_t v) {
    int index = head;
    while (tail == ((head + 1) % virtual_size)) { };
    enqueue(v);
  }

  uint64_t do_dequeue() {
    int index = tail;
    while (head == index) {};
    uint64_t ret = dequeue();
    return ret;
  }  
};

T_queue newq;

uint32_t dec2_fifo_init(uint32_t count, uint32_t size) {
  int size_table[8] = {8, 12, 32, 48, 64, 80, 96, 128}; 
  int actual_size = size_table[size];
  newq.init(actual_size);
  return count;
}

// Produce/Consume
void dec2_produce32(uint64_t qid, uint32_t data) {
  newq.do_enqueue(data);
}

void dec2_produce64(uint64_t qid, uint64_t data) {
  newq.do_enqueue(data);
}

uint32_t dec2_consume32(uint64_t qid) {
  return newq.do_dequeue();
}

uint64_t dec2_consume64(uint64_t qid) {
  return newq.do_dequeue();  
}

void dec2_load32_async(uint64_t qid, uint32_t *addr) {
  dec_produce32(qid, *addr);
}

void dec2_load64_async(uint64_t qid, uint64_t *addr) {
  dec_produce64(qid, *addr);
}
