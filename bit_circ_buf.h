// Bit push byte pull circular buffer

#ifndef _BIT_CIRC_BUF_H
#define _BIT_CIRC_BUF_H 1

#include <linux/kernel.h>

struct bit_circ_buf {
  char *buf;
  int size;
  int head;
  int tail;
};

void push_bit_to_cbuf(struct bit_circ_buf *cb, char value);
char pull_byte_from_cbuf(struct bit_circ_buf *cb);
int cbuf_size(struct bit_circ_buf *cb);
void print_cbuf_state(struct bit_circ_buf *cb);

#endif /* _BIT_CIRC_BUF_H */
