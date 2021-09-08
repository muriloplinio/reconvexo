#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#define MAX 6

// declaração da struct pessoa
typedef struct rcx_parcela
{
	int id;
    time_t datahora_on;
    time_t datahora_off;
} t_rcx_parcela;

void escrever_arquivo(t_rcx_parcela rcx_parcelas[]);
int ler_arquivo(t_rcx_parcela rcx_parcelas[MAX]);