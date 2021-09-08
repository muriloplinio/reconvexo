/*
	Gravar e carregar struct do arquivo
*/

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "reg.h"

// função para escrever os elementos de uma struct no arquivo
void escrever_arquivo(t_rcx_parcela rcx_parcelas[])
{

	int i;
	FILE * arq;

	// abre o arquivo para escrita no modo append (adiciona ao final)
	arq = fopen("dados.bin", "wb");
    if (arq != NULL) {
        i = fwrite(rcx_parcelas, sizeof(t_rcx_parcela), MAX, arq);
        if (i == MAX)
            printf("Gravacao %d registros com sucesso\n", i);
        else
            printf("Foram gravados apenas %d elementos\n", i);
        fclose(arq);
    }
    else
        puts("Erro: abertura do arquivo");

}

// função para ler do arquivo
// recebe o vetor que ela irá preencher
// retorna a quantidade de elementos preenchidos
int ler_arquivo(t_rcx_parcela rcx_parcelas[MAX])
{
	// abre o arquivo para leitura
	FILE * arq = fopen("dados.bin", "rb");
	if(arq != NULL)
	{
        t_rcx_parcela p;
		int ret = fread(rcx_parcelas, sizeof(t_rcx_parcela), MAX, arq);
        if (ret == MAX) {
            printf("Lidos %d registros com sucesso\n", ret);
        }
        else
            printf("Foram lidos apenas %d elementos\n", ret);
        fclose(arq);
    }
    else
        puts("Erro: abertura do arquivo");
	return(MAX);
}

int main(int argc, char *argv[])
{
  struct tm *data_hora_atual;     
  
  //variável do tipo time_t para armazenar o tempo em segundos  
  time_t segundos;
  
  //obtendo o tempo em segundos  
  time(&segundos);   
  
  //para converter de segundos para o tempo local  
  //utilizamos a função localtime  
  data_hora_atual = localtime(&segundos);  
    
    // vetor que será escrito no arquivo
	t_rcx_parcela rcx_parcelas[] = {{1,0,0}, 
									{2,segundos,segundos}, 
									{3,segundos,segundos},
									{4,segundos,segundos}, 
									{5,segundos,segundos}, 
									{6,segundos,segundos}
									};

	int dd = sizeof(rcx_parcelas) / sizeof(rcx_parcelas[0]);
	escrever_arquivo(rcx_parcelas);

	// vetor para onde serão carregados os dados
	// esse vetor foi criado para demonstrar que realmente funciona,
	// mas basta utilizar somente um vetor
	t_rcx_parcela aux_rcx_parcelas[MAX];

	int len_vet = ler_arquivo(aux_rcx_parcelas);
	int i;
	// mostrando os elementos do vetor aux_pessoas
	len_vet = 6;
	for(i = 0; i < len_vet; i++)
	{
		printf("Id: %d\n", aux_rcx_parcelas[i].id);
		printf("Idatahora_on: %x\n", aux_rcx_parcelas[i].datahora_on);
        printf("datahora_off: %x\n\n", aux_rcx_parcelas[i].datahora_off);
	}

	return 0;
}