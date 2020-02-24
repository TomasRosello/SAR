// PROGRAMA BASE PARA PLATAFORMA MÓVIL

#define CMD 0x00
#define GET_VER 0x29
#define GET_ENC_LEFT 0x23
#define GET_ENC_RIGHT 0x24
#define GET_ENCODERS 0x25
#define GET_V 0x26
#define GET_VI 0x2c
#define GET_ERROR 0x2d
#define SET_ACCEL 0x33
#define SET_POW_LEFT 0x31
#define SET_POW_RIGHT 0x32
#define SET_MODE 0x34
#define GET_CURR_LEFT 0x27
#define GET_CURR_RIGHT 0x28
#define ENABLE_REGUL 0x37
#define DISABLE_REGUL 0x36
#define ENABLE_TIMEOUT 0x39
#define DISABLE_TIMEOUT 0x38
#define CLEAR_ENC 0x35
#define MOTOR_LEFT 1
#define MOTOR_RIGHT 2
#define RETARDO 20 // periodo de muestreo T (Puede ser este u otro...)
#define MAX_POT 255 // máxima potencia a enviar
#define BASE_POT 128 // potencia nula
#define BAUD_0 19200
#define BAUD_1 38400 // velocidad de comunicación con la controladora
#define VEL_MAX 40 // velocidad máxima (pulsos/T)

int VELOC_IZQ=10; // velocidades deseadas para las ruedas (en pulsos/T)
int VELOC_DER=10;

float d=12.5; // Diámetro de ruedas en cm
float D=33; // Distancia entre ruedas en cm
int PPV=980; // pulsos de encoder por vuelta de rueda
byte accel=2; // aceleración (se alcanza la velocidad deseada en 1.024 seg)
byte error; // código de error devuelto por la controladora
int error_total=0;
byte ver; // Versión del software de la controladora
byte tension; // tensión de las baterías
byte motor; // MOTOR_LEFT o MOTOR_RIGHT
byte modo_control=0; // modo de control de los motores

// (0: control independiente de los motores)

//**********************************************************************
void setup()
{
 Serial.begin(BAUD_0);
 Serial1.begin(BAUD_1);
 
 PonerModo(modo_control);
 HabilitarTimeout();
 DeshabilitarRegulacion();
 PonerAceleracion(accel);
 BorrarEncoders();
 Avanzar(VELOC_IZQ,VELOC_DER); // ESTO ESTÁ AQUÍ PROVISIONALMENTE !!!
}

//***********************************************************************
byte LeerError()
{
 byte error;

 Serial1.write((byte)CMD);
 Serial1.write(GET_ERROR);
 
 delay(RETARDO);
 
 if (Serial1.available()>0)
   error=Serial1.read();
 
 return(error);
}

//************************************************************************
void BorrarEncoders()
{
  Serial1.write((byte)CMD);
  Serial1.write(CLEAR_ENC);
  
  return;
}

//************************************************************************
void PonerModo(byte modo)
{
  Serial1.write((byte)CMD);
  Serial1.write(SET_MODE);
  Serial1.write(modo);
 
  return;
}

//************************************************************************
void HabilitarRegulacion()
{
  Serial1.write((byte)CMD);
  Serial1.write(ENABLE_REGUL);
  
  return;
}

//************************************************************************
void HabilitarTimeout()
{
  Serial1.write((byte)CMD);
  Serial1.write(ENABLE_TIMEOUT);
 
  return;
}

//************************************************************************
void DeshabilitarTimeout()
{
  Serial1.write((byte)CMD);
  Serial1.write(DISABLE_TIMEOUT); 
  
  return;
}

//************************************************************************
void DeshabilitarRegulacion()
{
  Serial1.write((byte)CMD);
  Serial1.write(DISABLE_REGUL);
 
  return;
}

//************************************************************************
void PonerAceleracion(byte acel)
{
  Serial1.write((byte)CMD);
  Serial1.write(SET_ACCEL);
  Serial1.write(acel);
 
  return;
}

//***********************************************************************
void PonerPotencia(byte motor,byte pot)
{
 Serial1.write((byte)CMD);
 
 if (motor==MOTOR_LEFT)
   Serial1.write(SET_POW_LEFT);
 else
   Serial1.write(SET_POW_RIGHT);
 
 Serial1.write(pot);
 
 return;
}

//*************************************************************************
byte LeerVersion()
{
 byte vers;
 
 Serial1.write((byte)CMD);
 Serial1.write(GET_VER); 
 
 delay(RETARDO);
 
 if (Serial1.available()>0)
   vers=Serial1.read();
 
 return(vers);
}

//*************************************************************************
byte LeerTension()
{
 byte tens;
 
 Serial1.write((byte)CMD);
 Serial1.write(GET_V);
 
 delay(RETARDO);
 
 if (Serial1.available()>0)
   tens=Serial1.read();

 return(tens);
}

//************************************************************************
byte LeerCorriente(byte motor)
{
 byte curr;
 
 Serial1.write((byte)CMD);
 if (motor==MOTOR_LEFT)
   Serial1.write(GET_CURR_LEFT);
 else
   Serial1.write(GET_CURR_RIGHT);

 delay(RETARDO);
 
 if (Serial1.available()>0)
   curr=Serial1.read();
 
 return(curr);
} 

//*************************************************************************
long LeerVelocidad(byte motor)
{
 long valor=0;

 Serial1.write((byte)CMD);
 if (motor==MOTOR_LEFT)
   Serial1.write(GET_ENC_LEFT);
 else
   Serial1.write(GET_ENC_RIGHT);
 
 delay(RETARDO);
 
 if (Serial1.available()>3)
 {
   valor=Serial1.read()<<24;
   valor+=Serial1.read()<<16;
   valor+=Serial1.read()<<8;
   valor+=Serial1.read();
 }
 return(valor);
}

//*************************************************************************
long LeerVelocidades(long &vel_der)
{
 long vel_izq=0L;
 Serial1.write((byte)CMD);
 Serial1.write(GET_ENCODERS);
 delay(RETARDO);
 if (Serial1.available()>7)
 {
   vel_izq=Serial1.read()<<24;
   vel_izq+=Serial1.read()<<16;
   vel_izq+=Serial1.read()<<8;
   vel_izq+=Serial1.read();
   vel_der=Serial1.read()<<24;
   vel_der+=Serial1.read()<<16;
   vel_der+=Serial1.read()<<8;
   vel_der+=Serial1.read();
 }
 return(vel_izq);
}

//****************************************************************************
void Avanzar(int V_izq, int V_der)
{
  int VELOC_IZQ_AUX;
  int err_izq,err_der;
  byte pot_izq, pot_der;
  float kp = 1.0;
  long int VELOC_AUX_DER=0;
  
  VELOC_IZQ_AUX=LeerVelocidades(VELOC_AUX_DER);
  BorrarEncoders();
  err_izq=V_izq-VELOC_IZQ_AUX;
  err_der= V_der-VELOC_AUX_DER;

  pot_izq= kp * err_izq + BASE_POT;
  pot_der= kp * err_der + BASE_POT;
  
  
  if(pot_izq > 255)
    pot_izq=MAX_POT;
  if(pot_izq < 0)
    pot_izq=-MAX_POT;

    
  if(pot_der > 255)
    pot_der=255;
  if(pot_der < 0)
    pot_der=-MAX_POT;
   
  
  PonerPotencia(MOTOR_LEFT,pot_izq);
  PonerPotencia(MOTOR_RIGHT,pot_der);
  
  Serial.print("Izquierda");
  Serial.println(pot_izq);
  Serial.print("Derecha");
  Serial.println(pot_der);

  
}

//**********************************************************************
void Rotar(char sentido,int ang) // sentido ---> i: izquierdas; d: derechas
{
}

//**********************************************************************
void LeerSensores()
{
}

//**********************************************************************
void ProcesarSensores()
{
}

//**********************************************************************
void loop()
{
  Avanzar(VELOC_IZQ,VELOC_DER);
  delay(RETARDO);
}
