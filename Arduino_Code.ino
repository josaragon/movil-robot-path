#include <math.h>

//Incluyo libreria ROS y tipo de mensaje del topico
    #include <ros.h>
    #include <geometry_msgs/PoseWithCovarianceStamped.h>
    
//Pines de entrada digital asociados a la lectura de los encoders
int encoder_left_0 = 18;
int encoder_left_1 = 19;
int encoder_right_0 = 20;
int encoder_right_1 = 21;

//Pines de salida PWM para la regulación de la velocidad de los motores
#define right_vel 8
#define left_vel 10

//Pines de salida digital para el sentido de giro de los motores
#define right_positive 30
#define right_negative 31
#define left_positive 28
#define left_negative 29

//Variables donde se guardarán el número de interrupciones que generan los encoders
int ticks_left = 0;
int ticks_right = 0;

//Velocidad calculada de ambas ruedas
float speed_left;
float speed_right;

//Radio de las ruedas y distancia entre ellas
double R = 0.03;
double b = 0.0915;

//Coordenadas robot y punto destino
double phi;
double phi_inc;
double phi_prox, phi_calc;
double phi_dif, phi_dif_abs;
double x, y, s, v_med, s_inc;
double x_obj[2] = { -0.5, 0.5};
double y_obj[2] = {0, 0};
double x_ros,y_ros;

//Variables para calcular los tiempos de ciclos
double t_actual = 0;
double t_anterior = 0;
double t_inc = 0;
int i,k;

//Creo el nodo para la comunicacion con ROS   
    ros::NodeHandle nh;

//Funcion a llamar para almacenar las variables del topico al que nos hemos subscrito
   void messageCb( const geometry_msgs::PoseWithCovarianceStamped& punto){
     x_obj[k] = punto.pose.pose.position.x;
     y_obj[k] = punto.pose.pose.position.y;
     k=k+1;
   }

//Subscripcion al topico en el que esta la variable que nos interesa
   ros::Subscriber<geometry_msgs::PoseWithCovarianceStamped> sub("amcl_pose", &messageCb );


//Inicialización de las configuraciones y variables
void setup() {

  nh.initNode();
  nh.subscribe(sub);
  phi_inc = 0;
  phi = 0;
  phi_calc = 0;
  phi_prox = 0;
  phi_dif = 0;
  phi_dif_abs = 0;

  //Punto obj
  s = 0;
  s_inc = 0;
  v_med = 0;

  //Tiempo bucle
  t_actual = 0;
  t_anterior = 0;
  t_inc = 0;

  pinMode(left_positive, OUTPUT);
  pinMode(right_positive, OUTPUT);
  pinMode(left_negative, OUTPUT);
  pinMode(right_negative, OUTPUT);
  pinMode(left_vel, OUTPUT);
  pinMode(right_vel, OUTPUT);

  pinMode(encoder_left_0, INPUT);
  pinMode(encoder_left_1, INPUT);
  pinMode(encoder_right_0, INPUT);
  pinMode(encoder_right_1, INPUT);

  //Asociamos a las entradas digitales de los encoders, interrupciones de tipo "change"
  attachInterrupt(5, ticks_left_0, CHANGE);
  attachInterrupt(4, ticks_left_1, CHANGE);
  attachInterrupt(3, ticks_right_0, CHANGE);
  attachInterrupt(2, ticks_right_1, CHANGE);

  Serial.begin(115200);
}

///////////////// Funciones asociadas a las interrupciones generadas por los encoders ///////////////////
void ticks_left_0() {
  if (digitalRead(encoder_left_0) == digitalRead(encoder_left_1))
    ticks_left++;
  else
    ticks_left--;
}

void ticks_left_1() {
  if (digitalRead(encoder_left_0) == digitalRead(encoder_left_1))
    ticks_left--;
  else
    ticks_left++;
}


void ticks_right_0() {
  if (digitalRead(encoder_right_0) == digitalRead(encoder_right_1)) {
    ticks_right++;
  } else {
    ticks_right--;
  }
}


void ticks_right_1() {
  if (digitalRead(encoder_right_0) == digitalRead(encoder_right_1)) {
    ticks_right--;
  } else {
    ticks_right++;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////

////////////////////////////////// Bucle principal del programa ////////////////////////////////////////

void loop() {

  //Se calcula el tiempo que lleva el programa ejecutándose
  t_actual = millis();

  //Se compara hasta que lleve mas de 100 ms aproximadamente, como muestreo de la lectura de velocidad y acciones del robot
  if ((t_actual - t_anterior) >= 100) {
    t_inc = t_actual - t_anterior;

    //Velocidad que ha llevado durante los últimos 100 ms
    speed_left = 0.005661 * ticks_left; //2*3.14159*3*(ticks_left/333)/0.1 = 0.56605273 cm/s o 0.0056605273 m/s
    speed_right = 0.005661 * ticks_right;
    ticks_left = 0;
    ticks_right = 0;

    ///////////////////// Cálculo de los ángulos actual y objetivo //////////////////////

    //Cálculo del ángulo en el que se encuentra el robot
    phi_inc = ((speed_right - speed_left) * 0.1) / (4 * b);
    phi_prox = phi_prox + phi_inc;

    //Nuevo punto objetivo del robot
    x = x_obj[i];
    y = y_obj[i];

    //Cálculo del angulo del punto objetivo
    phi_calc = atan(x / y);

    ///////// Cuadrante del angulo objetivo ///////////

    //Si el punto se encuentra en el tercer cuadrante, se suma pi radianes
    if ( x < 0 && y < 0 ) {
      phi_calc = phi_calc + 3.14159;
    }
    //Si el punto se encuentra en el cuart cuadrante, se resta pi radianes
    if ( x < 0 && y > 0 ) {
      //Sumo 180º
      phi_calc = phi_calc - 3.14159;
    }

    //Si el punto objetivo tiene coordenada nula, se rectifica
    if ( x == 0 ) {
      if (y > 0) {
        phi_calc = phi_calc + 1.5708;
      }
      if (y < 0) {
        phi_calc = phi_calc - 1.5708;
      }
      if (y == 0) {
        phi_calc = 0;
      }
    }
    if ( y == 0) {
      if (x > 0) {
        phi_calc = phi_calc - 1.5708;
      }
      if (x < 0) {
        phi_calc = phi_calc - 1.5708;
      }
    }

    ///////////////////////////////////////////////////

    //Cálculo de la diferencia entre el ángulo del punto objetivo y del actual
    phi_dif = phi_prox - phi_calc;
    phi_dif_abs = abs(phi_dif);

    //Si la diferencia es menor que 5 grados (0.08726 en radianes)
    if (phi_dif_abs < 0.08726) {

      //Cálculo de la velocidad media y de la distancia recorrida
      v_med = (speed_right + speed_left) / 2;
      s_inc = v_med * 0.1;
      s = s + s_inc;

      //Si además aún no ha llegado a su punto destino
      if (s < sqrt(pow(x, 2) + pow(y, 2))) {

        //Se activan los motores para que siga hacia delante
        digitalWrite(left_negative, LOW);
        digitalWrite(left_positive, HIGH);
        analogWrite(left_vel, 100);

        digitalWrite(right_negative, LOW);
        digitalWrite(right_positive, HIGH);
        analogWrite(right_vel, 100);
      }

      //Si ya ha alcanzado el punto destino
      else {

        //Se desactivan los motores
        digitalWrite(left_negative, LOW);
        digitalWrite(left_positive, HIGH);
        analogWrite(left_vel, 0);

        digitalWrite(right_negative, LOW);
        digitalWrite(right_positive, HIGH);        
        analogWrite(right_vel, 0);

        //Se reinicializan las variables necesarias para ir al siguiente punto objetivo
        phi_prox = 0;
        phi_dif = 0;
        s = 0;
        i = i + 1;
      }
    }

    //Si la diferencia entre el ángulo objetivo y el actual es mayor a 5 grados
    else {

      //Gira a la izquierda si la diferencia es positiva
      if (phi_dif > 0) {
        digitalWrite(left_negative, LOW);
        digitalWrite(left_positive, HIGH);
        analogWrite(left_vel, 200);

        digitalWrite(right_negative, HIGH);
        digitalWrite(right_positive, LOW);
        analogWrite(right_vel, 200);
      }

      //Gira a la derecha si la diferencia es negativa
      else {
        digitalWrite(left_negative, HIGH);
        digitalWrite(left_positive, LOW);
        analogWrite(left_vel, 200);

        digitalWrite(right_negative, LOW);
        digitalWrite(right_positive, HIGH);
        analogWrite(right_vel, 200);
      }

      delay(100);

    }
    t_anterior = t_actual;
  }
}

