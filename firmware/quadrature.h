#ifndef XRP_QUADRATURE_H
#define XRP_QUADRATURE_H

void quadrature_setup_tim3(void);
void quadrature_init_tim3(int pos);
int quadrature_read_pos_tim3(void);

void quadrature_setup_tim2(void);
void quadrature_init_tim2(int pos);
int quadrature_read_pos_tim2(void);

#endif
