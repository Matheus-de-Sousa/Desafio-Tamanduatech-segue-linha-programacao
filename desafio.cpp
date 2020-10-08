#include <iostream>
using namespace std;

#define nSLat 4 //número de sensores laterais
#define nSarray 3 //número de sensores array

//Parâmetros de velocidade do segue-linha na reta 
#define velRetaMax 200.0
#define velRetaMin 50.0
#define velRetaBase 100.0

//Parâmetros de velocidade do segue-linha na curva 
#define velCurvaMax 175.0
#define velCurvaMin 40.0
#define velCurvaBase 75.0

//Parâmetros do PID do segue-linha na reta 
#define pidRetaSetpoint 2.0
#define pidRetaKp 1.00
#define pidRetaKi 0.10
#define pidRetaKd 0.30

//Parâmetros do PID do segue-linha na curva 
#define pidCurvaSetpoint 3.0
#define pidCurvaKp 0.80
#define pidCurvaKi 0.30
#define pidCurvaKd 0.60

//Valores máximos dos sensores laterais
#define maxSL1 10
#define maxSL2 15
#define maxSL3 17
#define maxSL4 12

//Valores mínimos dos sensores laterais
#define minSL1 5
#define minSL2 2
#define minSL3 3
#define minSL4 7

//Valores máximos dos sensores array
#define maxSA1 20
#define maxSA2 25
#define maxSA3 27

//Valores mínimos dos sensores array
#define minSA1 6
#define minSA2 9
#define minSA3 4

//Struct com dados relativos a velocidade do segue-linha
struct valuesSpeed{
    float velMotesq;
    float velMotdir;
    float curva[3]; //array com parâmetros da velocidade na curva; curva[0]=Max, curva[1]=Min, curva[2]=base
    float reta[3];  //array com parâmetros da velocidade na reta;  reta[0]=Max, reta[1]=Min, reta[2]=base
};

//Struct com dados relativos ao controle PID do segue-linha
struct valuesPID{
    float entrada;
    float saida;
    float curva[4]; //array com parâmetros do PID na curva;  curva[0]=setpoint, curva[1]=kp, curva[2]=ki, curva[3]=kd
    float reta[4];  //array com parâmetros do PID na reta;   reta[0]=setpoint, reta[1]=kp, reta[2]=ki, reta[3]=kd
};

//Struct com dados relativos às marcas
struct valuesMarks{
    short int qntdEsq;
    short int qntdDir;
};

//Struct com dados relativos aos encoders do segue-linha
struct valuesEncs{
    short int qntdEsq;
    short int qntdDir;
};

//Struct com dados relativos aos sensores laterais do segue-linha
struct valuesSLat{
    short int Svalues[nSLat]; //array com valores dos sensores laterais
    float mediaPond;
    short int maxValues[nSLat]; //array com valores máximos dos sensores laterais
    short int minValues[nSLat]; //array com valores mínimos dos sensores laterais
};

//Struct com dados relativos aos sensores array do segue-linha
struct valuesSarray{
    short int Svalues[nSarray]; //array com valores dos sensores array
    float mediaPond;
    short int maxValues[nSarray]; //array com valores máximos dos sensores array
    short int minValues[nSarray]; //array com valores mínimos dos sensores array
};

//classe com métodos e atributos para manipular os dados relacionados ao segue-linha
class carDados{ 
  public:
     uint8_t state;//0: parado, 1: linha, 2: curva
     uint32_t lastUpdate;
     float P=0,I=0,D=0; //atributos para o cálculo do PID
     float erroAnterior=0; //atributo para o cálculo do PID
	 
	 //variáveis de controle das structs criadas para manipular os dados do segue-linha
     valuesSpeed speed;
     valuesPID PID;
     valuesMarks marks;
     valuesEncs motEncs;
     valuesSLat sLat;
     valuesSarray sArray;
	 
	 //método construtor 
     carDados(uint8_t estado, uint32_t L_update){
       state = estado;
       lastUpdate = L_update;
     }
	 
	//método para calcular o valor do PID 
    void vTaskCalcularPid(float *input, float*output){
      P = *input;
      I += *input;
      D = *input - erroAnterior;
      if(state==1) *output = (PID.reta[1]*P) + (PID.reta[2]*I) + (PID.reta[3]*D); //cálculo do valor do PID na reta; PID.reta[1]=kp, PID.reta[2]=ki, PID.reta[3]=kd 
      if(state==2) *output = (PID.curva[1]*P) + (PID.curva[2]*I) + (PID.curva[3]*D); //cálculo do valor do PID na curva; PID.curva[1]=kp, PID.curva[2]=ki, PID.curva[3]=kd 
      erroAnterior = *input;
    }
	
	//método para setar os parâmetros da velocidade do segue-linha na reta
    void setParams_Velreta(float max,float min, float base){
      speed.reta[0] = max;
      speed.reta[1] = min;
      speed.reta[2] = base;
    }
	
	//método para setar os parâmetros da velocidade do segue-linha na curva
    void setParams_Velcurva(float max,float min, float base){
      speed.curva[0] = max;
      speed.curva[1] = min;
      speed.curva[2] = base;
    }
	
	//método para setar os parâmetros do controle PID do segue-linha na reta
    void setParams_PIDreta(float setpoint,float kp, float ki, float kd){
      PID.reta[0] = setpoint;
      PID.reta[1] = kp;
      PID.reta[2] = ki;
      PID.reta[3] = kd;
    }
	
	//método para setar os parâmetros do controle PID do segue-linha na curva
    void setParams_PIDcurva(float setpoint,float kp, float ki, float kd){
      PID.curva[0] = setpoint;
      PID.curva[1] = kp;
      PID.curva[2] = ki;
      PID.curva[3] = kd;
    }
	
	//método para setar os valores máximos dos sensores laterais do segue-linha
    void setParams_SLmaxValues(short int sl1,short int sl2, short int sl3, short int sl4){
      sLat.maxValues[0] = sl1;
      sLat.maxValues[1] = sl2;
      sLat.maxValues[2] = sl3;
      sLat.maxValues[3] = sl4;
    }
	
	//método para setar os valores mínimos dos sensores laterais do segue-linha
    void setParams_SLminValues(short int sl1,short int sl2, short int sl3, short int sl4){
      sLat.minValues[0] = sl1;
      sLat.minValues[1] = sl2;
      sLat.minValues[2] = sl3;
      sLat.minValues[3] = sl4;
    }
	
	//método para setar os valores máximos dos sensores array do segue-linha
    void setParams_SAmaxValues(short int sa1,short int sa2, short int sa3){
      sArray.maxValues[0] = sa1;
      sArray.maxValues[1] = sa2;
      sArray.maxValues[2] = sa3;
    }
	
	//método para setar os valores mínimos dos sensores array do segue-linha
    void setParams_SAminValues(short int sa1,short int sa2, short int sa3){
      sArray.minValues[0] = sa1;
      sArray.minValues[1] = sa2;
      sArray.minValues[2] = sa3;
    }
};
int main()
{
	//Inicializa todos os atributos e structs do objeto lineFollower com valores aleatórios com o objetivo de testar o código
    carDados lineFollower(0,0); //cria um objeto da classe carDados
    lineFollower.speed.velMotesq=20.12;
    lineFollower.speed.velMotdir=35.30;
    lineFollower.PID.entrada = 10.78;
    lineFollower.PID.saida=45.91;
    lineFollower.marks.qntdEsq=7;
    lineFollower.marks.qntdDir=6;
    lineFollower.motEncs.qntdEsq=50;
    lineFollower.motEncs.qntdDir=70;
    for(int i=0;i<nSLat;i++)lineFollower.sLat.Svalues[i]=i+11;
    lineFollower.sLat.mediaPond=67.83;
    for(int i=0;i<nSarray;i++)lineFollower.sArray.Svalues[i]=i+21;
    lineFollower.sArray.mediaPond=47.53;
    
    //Inicializa todos os Parâmetros do objeto lineFollower, através dos métodos criados para esta tarefa, com valores aleatórios para testar o código
    lineFollower.setParams_Velreta(velRetaMax,velRetaMin,velRetaBase);
    lineFollower.setParams_Velcurva(velCurvaMax,velCurvaMin,velCurvaBase);
    lineFollower.setParams_PIDreta(pidRetaSetpoint,pidRetaKp,pidRetaKi,pidRetaKd);
    lineFollower.setParams_PIDcurva(pidCurvaSetpoint,pidCurvaKp,pidCurvaKi,pidCurvaKd);
    lineFollower.setParams_SLmaxValues(maxSL1,maxSL2,maxSL3,maxSL4);
    lineFollower.setParams_SLminValues(minSL1,minSL2,minSL3,minSL4);
    lineFollower.setParams_SAmaxValues(maxSA1,maxSA2,maxSA3);
    lineFollower.setParams_SAminValues(minSA1,minSA2,minSA3);
    
	
	//Exibição dos dados do segue-linha como teste do funcionamento do código
    cout<<"---------- Dados(valores) do segue linha ----------\n\n";
    cout<<"State:"<<int(lineFollower.state)<<endl;
    cout<<"lastUpdate:"<<lineFollower.lastUpdate<<endl;
    cout<<"Velocidade motor esquerdo:"<<lineFollower.speed.velMotesq<<endl;
    cout<<"Velocidade motor direito:"<<lineFollower.speed.velMotdir<<endl;
    cout<<"Entrada do PID:"<<lineFollower.PID.entrada<<endl;
    cout<<"Saida do PID:"<<lineFollower.PID.saida<<endl;
    cout<<"Qntd. de marcas esquerda:"<<lineFollower.marks.qntdEsq<<endl;
    cout<<"Qntd. de marcas direita:"<<lineFollower.marks.qntdDir<<endl;
    cout<<"Qntd. motEncoder esquerda:"<<lineFollower.motEncs.qntdEsq<<endl;
    cout<<"Qntd. motEncoder direita:"<<lineFollower.motEncs.qntdDir<<endl;
    for(int i=0;i<nSLat;i++) cout<<"Valor do sensor SLat"<<i+1<<":"<<lineFollower.sLat.Svalues[i]<<endl;
    cout<<"Media ponderada dos sensores laterais:"<<lineFollower.sLat.mediaPond<<endl;
    for(int i=0;i<nSarray;i++) cout<<"Valor do sensor Sarray"<<i+1<<":"<<lineFollower.sArray.Svalues[i]<<endl;
    cout<<"Media ponderada dos sensores array:"<<lineFollower.sArray.mediaPond<<endl;
    
    cout<<"\n---------- Parâmetros do segue linha -----------\n\n";
    cout<<"Velocidade maxima na reta:"<<lineFollower.speed.reta[0]<<endl;
    cout<<"Velocidade minima na reta:"<<lineFollower.speed.reta[1]<<endl;
    cout<<"Velocidade base na reta:"<<lineFollower.speed.reta[2]<<endl;
    cout<<"Velocidade maxima na curva:"<<lineFollower.speed.curva[0]<<endl;
    cout<<"Velocidade minima na curva:"<<lineFollower.speed.curva[1]<<endl;
    cout<<"Velocidade base na curva:"<<lineFollower.speed.curva[2]<<endl;
    cout<<"PID setpoint na reta:"<<lineFollower.PID.reta[0]<<endl;
    cout<<"PID Kp na reta:"<<lineFollower.PID.reta[1]<<endl;
    cout<<"PID Ki na reta:"<<lineFollower.PID.reta[2]<<endl;
    cout<<"PID Kd na reta:"<<lineFollower.PID.reta[3]<<endl;
    cout<<"PID setpoint na curva:"<<lineFollower.PID.curva[0]<<endl;
    cout<<"PID Kp na curva:"<<lineFollower.PID.curva[1]<<endl;
    cout<<"PID Ki na curva:"<<lineFollower.PID.curva[2]<<endl;
    cout<<"PID Kd na curva:"<<lineFollower.PID.curva[3]<<endl;
    for(int i=0;i<nSLat;i++) cout<<"Valor maximo do sensor SLat"<<i+1<<":"<<lineFollower.sLat.maxValues[i]<<endl;
    for(int i=0;i<nSLat;i++) cout<<"Valor minimo do sensor SLat"<<i+1<<":"<<lineFollower.sLat.minValues[i]<<endl;
    for(int i=0;i<nSarray;i++) cout<<"Valor maximo do sensor Sarray"<<i+1<<":"<<lineFollower.sArray.maxValues[i]<<endl;
    for(int i=0;i<nSarray;i++) cout<<"Valor minimo do sensor Sarray"<<i+1<<":"<<lineFollower.sArray.minValues[i]<<endl;
    
    
	//Teste do método para calcular o PID do segue-linha e exibição dos resultados
    cout<<"\n---------- Função de cálculo do PID do segue linha ----------\n\n";
    lineFollower.state=1; //estado em que o segue-linha está na reta para cálculo do PID na reta
    lineFollower.vTaskCalcularPid(&lineFollower.PID.entrada,&lineFollower.PID.saida); //passagem dos argumentos para os ponteiros e cálculo do PID na reta
    cout<<"Resultado do calculo do PID na reta:"<<lineFollower.PID.saida<<endl;
    lineFollower.state=2; // estado em que o segue-linha está na curva para o cálculo do PID na curva
	//reset das variáveis relativas ao PID
    lineFollower.P=0;
    lineFollower.I=0;
    lineFollower.D=0;
    lineFollower.erroAnterior=0;
    lineFollower.vTaskCalcularPid(&lineFollower.PID.entrada,&lineFollower.PID.saida); //Cálculo do PID na curva
    cout<<"Resultado do calculo do PID na curva:"<<lineFollower.PID.saida<<endl;
    return 0;
    
    
}