int LED = 13;
int  distx = 0, disty = 0; //x -coordinate  y coordinate calculation
float encoderdistance = 0;
float x_bar=1,y_bar=1;
int v = 1; //no. of nodes
int r = 0; // how many times should be visited or not
int count1, count2, count3; // to store whether there is left or forward or right
int endpoint = 0;
int found_left = 0;
int found_right = 0;
int found_straight = 0;
int p;
int turn45=0;
//MOTOR
int rm, lm;
int in1_R = 6;                                                                                      //      a[0]  a[1] a[2] a[3] a[4] a[5] a[6]  fs
int in2_R = 7;                                                            // forward sensor orientation:    0    0    0     1    1    1    0         0
int in1_L = 4;
int in2_L = 5;
int d=0;
//SENSOR
int IRpins[] = {28, 9, 30, 31, 32, 33 , 34};
int a[8];
int front_sensor = 36;
int fs;

//PID
int initSpeed = 255;
int adjSpeed;
int integral = 0;
int leftMotor, rightMotor;
int pos;
int lastError = 0;
double pidValue = 0;
double error = 0, derivative = 0;
char dir = ' ';
float Kp = 1.45 / 1.0;
float Ki = 0.0;
float Kd = 1.1;
int right = 0;
int left = 0;

//TIMER
int STtime = 0;
int SPtime = 0;
int Tdelay = 0;

struct node
{
  int count1, count2, count3, r;
  int distx = 0, disty = 0;
  node* next;
};

struct adjlist
{
  node*  head;
};

struct graph
{
  int v;   // number of vertices
  adjlist* arr[];
};

struct node* updatenode(struct graph *g, int v, int distx, int disty, int count1, int count2, int count3, int r,int d) {

  if ((g->arr[v]->head == NULL ) && d==0)
  {
    Serial.print("NODE CREATED ..........");

    node* newnode = (node*)malloc(sizeof(node));

    newnode->count1 = count1;
    newnode->count2 = count2;
    newnode->count3 = count3;
    newnode->distx;
    newnode->disty;
    newnode->r;
    newnode->next = NULL;
    //g->arr[v]->head = newnode;
   // Serial.println( g->arr[v]->head->count3);
    //printgraph(g);
    //creategraph(v);
    return newnode;
  }
  else
  {
    if (g->arr[v]->head->count1 == 1)
    {
      g->arr[v]->head->count1 = 0;
      g->arr[v]->head->r = r--;
    }
    else if (g ->arr[v]->head->count2 == 1)
    {
      g->arr[v]->head->count2 = 0;
      g->arr[v]->head->r = r--;
    }
    else if ( g->arr[v]->head->count3 == 1)
    {
      g->arr[v]->head->count3 = 0;
      g->arr[v]->head->r = r--;
    }
  }
};
void junction(struct  graph* g, int v,  int distx, int disty, int count1, int count2, int count3, int r, int d)
{ 
  node* newnode = updatenode(g, v, distx, disty, count1, count2, count3, r,d); 
  v=++v;
  creategraph(v);
  newnode->next = g->arr[v]->head; 
  g->arr[v]->head = newnode;  
}

struct graph* creategraph(int v)
{
  struct graph* g = (struct graph*)malloc(sizeof(graph));
  g->v = v;
  g->arr[v] = (adjlist*)malloc(sizeof(adjlist) * v);
  g->arr[v]->head = NULL;
  return g;
};

void rotateright()
{
  Serial.println("...............................ROtating RIGHT...............................");
  //Serial.println("_____________________________ RIGHT_TURN");
  analogWrite(in1_R, LOW); //OVERSHOOT
  analogWrite(in2_R, 70);
  analogWrite(in1_L, 254);
  analogWrite(in2_L, LOW);
  sensor();
  while (a[1] != 1) // wait for outer most sensor to find the line
    sensor();

  Serial.println("+++++++++++++++++++++++++++++++++++++++++++");
  // slow down speed
  analogWrite(in1_R, LOW); //OVERSHOOT
  analogWrite(in2_R, 90);
  analogWrite(in1_L, 150);
  analogWrite(in2_L, LOW);
  sensor();
  // find center
  while (pos > 450 && pos <= -1)  // tune - wait for line position to find near center
  {
    sensor();
    Serial.println(pos);
  }
  Serial.println("++++++++END POS RIGHT++++++++++");
  // stop both motors
  analogWrite(in1_R, LOW); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, LOW);
  // delay(50000);
}

void rotateleft()
{
  Serial.println("................................Rotating LEFT.................................");
 // Serial.println("_____________________________ LEFT_TURN");
  analogWrite(in1_R, 250); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, 120);
  sensor();
  while (a[0] != 1)  // wait for outer most sensor to find the line
    sensor();

  Serial.println("----------------------------------------");
  // slow down speed
  analogWrite(in1_R, 100); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, 50);
  sensor();
  // find center
  while ( pos < 100 )  // tune - wait for line position to find near center
  {
    sensor();
    Serial.println(pos);
  }
  Serial.println("++++++++END POS LEFT++++++++++");
  // stop both motors
  analogWrite(in1_R, LOW); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, LOW);
  //delay(50000);
  sensor();
}

void rotate(int thetha)
{
  Serial.println("................RotatingCoordinates........................");
  x_bar = cos(thetha) - sin(thetha);
  y_bar = sin(thetha) + cos(thetha);
}

//float encoder(){
//  return encoderdistance;
//}

int motiondistance(graph* g)
{
  if ((x_bar == 1 && y_bar == -1 ) || (x_bar == -1 && y_bar == 1))
    distx = distx + (encoderdistance * x_bar);

  else if ((x_bar == 1 && y_bar == 1) || (x_bar == -1 && y_bar == -1))
    disty = disty + (encoderdistance * x_bar);

  //updatenode(g, v, distx, disty, count1, count2, count3, r,d);
  //r = 0;
  //encoderdistance = 0;
}

int sensor()
{
  for (int j = 0; j < 7; j++)
    a[j] = digitalRead(IRpins[j]);
  fs = !digitalRead(front_sensor);
  for (int j = 0; j < 7; j++)
  {
    Serial.print(a[j]);
    Serial.print("  ");
  }
  Serial.print(fs);
  Serial.println();
  pos = ((100 * a[2]) + (200 * a[3]) + (300 * a[4]) + (400 * a[5]) + (500 * a[6])) / (a[2] + a[3] + a[4] + a[5] + a[6]);
  Serial.println(pos);
}

int lfrpriority(int a[], int i, graph* g)
{
 loopdetect(g);
  if (found_left == 1)
  {
    //marks the left track going as 1
    rotateleft();   //point rotation to left
    if(turn45==1) rotate(45);
    else rotate(90);
    //count1--;
  }
  else if (found_straight == 1)
  {
    //count2--;   //marks the straight track going as 1 acc to lfr
    pid(g, count1, count2, count3);
  }
  else if (found_right == 1)
  {
    // count3--;
    rotateright();
    if(turn45==1) rotate(-45);
    else rotate(-90);
    turn45=0;
  }
  motiondistance(g);
  
  //encoderdistance = 0;
  if (found_left == 1)
  {
    r++;
    count1++;
    Serial.println("left path Entered into the info");
  }
  if (found_straight == 1)
  {
    r++;
    count2++;
    Serial.println("forwardpath entered into the info");
  }
  if (found_right == 1)
  {
    r++;
    count3++;
    Serial.println(" right path entered into the info");
  }
  ++v;
  Serial.println(count1);
  Serial.println(count2);
  Serial.println(count3);
  Serial.println(v);
  junction(g, v, distx, disty, count1, count2, count3, r,d);
  count1=0;
  count2=0;
  count3=0;
  encoderdistance = 0;
  pid(g, count1, count2, count3);
}

int pid(graph *g, int count1, int count2, int count3)
{
  Serial.println("PID initializing..........");
  //STtime = millis();
  lastError = 0;
  integral = 0;
  adjSpeed = 0;
  while (1)
  { 
    //calc_dis();
    sensor();
    if ( (a[0] == 0 &&  a[1] == 0 && a[2] == 0 &&  a[3] == 0 && a[4] == 0 && a[5] == 0 && a[6] == 0 && fs == 1) ) //|| (cm <= 13 && cm > 0)
    {
      Serial.println("++++++++++++DEAD_END++++++++++++");
      p=1;
      deadend(g, count1, count2, count3);
      //dir = 'B';
    }
    if ( a[0] == 1 || a[1] == 1  )
    {
      Serial.println("++++++++++++TURN++++++++++++");
      // dir = ' ';
      p = 2;
      turn(a,  8, g);
     
    }
    error = pos - 300;
    derivative = (error - lastError);
    integral = (error + integral);
    if (integral > 255) {
       integral = 255;
    }
    if (integral < -255) {
      integral = -255;
    }
    pidValue = (Kp * error) + (Kd * derivative) + (Ki * integral);
    lastError = error;
    adjSpeed = pidValue;
    //      Serial.print(pidValue); Serial.print("    ");
    //      Serial.println(adjSpeed);
    leftMotor =  initSpeed + adjSpeed;
    rightMotor = initSpeed - adjSpeed;
    if (leftMotor > 255) 
      leftMotor = 255;
    
    if (leftMotor < -255) 
      leftMotor = -255;
    
    if (rightMotor > 255) {
      rightMotor = 255;
    }
    
    if (rightMotor < -255) 
      rightMotor = -255;
    
    if (leftMotor < 0)
    { 
      digitalWrite(in1_L, LOW);
      lm = in2_L;
    }
    else
    { 
      digitalWrite(in2_L, LOW);
      lm = in1_L;
    }
    
    if (rightMotor < 0)
    { 
      digitalWrite(in1_R, LOW);
      rm = in2_R;
    }
    else
    { 
      digitalWrite(in2_R, LOW);
      rm = in1_R;
    }
    Serial.print(leftMotor);
    Serial.print("  ");
    Serial.println(rightMotor);
    analogWrite(rm, abs(rightMotor));
    analogWrite(lm, abs(leftMotor));
  }
}

//void  motion(graph *g, int count1, int count2, int count3)
//{
//
//  pid(g, count1, count2, count3);
//  Serial.println("forwardmotion completed");
//  motiondistance(g);
//  Serial.println("distance stored as coordinates");
//}


int loopdetect(graph* g)
{
  if(g->arr[v]->head->distx!=0 || g->arr[v]->head->disty!=0)
  {
    back();
    v=--v;
    g->arr[v]->head;
  }
}


//void brake()
//{
//  Serial.println(" ..................................applying brake");
//  analogWrite(in1_R, LOW);
//  analogWrite(in2_R, LOW);
//  analogWrite(in1_L, LOW);
//  analogWrite(in2_L, LOW);
//  //      delay(500);
//}



void back()
{
  Serial.println("_____________________________BACK_TURN");
  analogWrite(in1_R, 255); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, 255);
  analogWrite(in2_L, LOW);
  delay(280);
  analogWrite(in1_R, 200);
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, 200);
  sensor();
  while (a[0]!= 1)  // wait for outer most sensor to find the line
  {
    sensor();
  }
  Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  // slow down speed
  analogWrite(in1_R, 100); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, 100);
  sensor();
  // find center
  while ( pos <= 100 )  // tune - wait for line position to find near center
  {
    sensor();
    Serial.println(pos);
  }
  Serial.println("++++++++END POS BACK++++++++++");
  // stop both motors
  analogWrite(in1_R, LOW); //OVERSHOOT
  analogWrite(in2_R, LOW);
  analogWrite(in1_L, LOW);
  analogWrite(in2_L, LOW);
}

int deadend(graph *g, int count1, int count2, int count3)
{
  Serial.println("Deadend detected");
  back();
  rotate(180);
  //display nullto  address of that node
  g->arr[v]->head = NULL;
  v = --v;
  g->arr[v]->head;
  d=1;
  updatenode(g, v, distx, disty, count1, count2, count3, r,d);
  d=0;
  //Serial.println("Deadend");
    if(g->arr[v]->head->count2==0)
  //  {
  //    if(x_bar==1 && y_bar==1)
  //
  //    rotateleft();
  //    rotate(90);
  //  }
  //  else if(x_bar==-1 && y_bar==1)
  //  {
  //
  //    rotateright();
  //    rotate(-90);
  //  }
  //
  //  if(g->arr[v]->head-> count2==1)
  //  {
  //    if(x_bar==1 && y_bar==1)
  //
  //    rotateleft();
  //    rotate(90);
  //  }
  //  else if(x_bar==-1 && y_bar==1)
  //  {
  //
  //    rotateright();
  //    rotate(-90);
  //  }
  // }
  //  else
  //  {
  //       continue;//if onthe left side      rotate right if on the positive x axis
  //  }
  pid(g, count1, count2, count3);
}

//int forwardmotion(graph *g, int count1, int count2, int count3)
//{
//  Serial.println("inside forward motion");
//  //while (fs == 0 && a[0] == 0 && a[1] == 0)
//  
//    Serial.print("Satisfied condition");
//    motion(g, count1, count2, count3);
//    Serial.print("motion completed");
//}

void turn(int a[], int i, graph* g)
{
  if (p == 2)
    {
       found_left = 0;
       found_straight = 0;
       found_right = 0;
      if (a[0] == 1)
      { 
        found_left = 1;
        sensor();
        if (a[1] == 1)
          found_right = 1;
      }
      else if(a[1]==1)
      {
        found_right = 1;
        sensor();
        if (a[0] == 1)
        found_left = 1;
      }
      analogWrite(in1_R, 255); //OVERSHOOT
      analogWrite(in2_R, LOW);
      analogWrite(in1_L, 255);
      analogWrite(in2_L, LOW);
      //delay(270);
      int timenow = millis();
      while (millis() - timenow < 75)
      {
        //////////////////////
        if (a[1] == 1)
          {
            found_right = 1;
            turn45=1;
          
          }
        if (a[0] == 1)
        {
          found_left = 1;
          turn45=1;
        }
        //////////////////////
        sensor();
      }
      if ((found_left == 1 || found_right == 1) ) // && cm > 17
      { if (fs == 0 ) // STRAIGHT CONDITION
          found_straight = 1;
      }
      Serial.println(found_left);
      Serial.println(found_straight);
      Serial.println(found_right);
      lfrpriority(a, 8, g);
    }
}
void start(graph* g, int count1, int count2, int count3 )
{
  while (1)
  {
    pid(g, count1, count2, count3);
    if (p == 1)
      deadend(g, count1, count2, count3);  //a[0]=left sensor  a[1]=leftboundary  a[2]=front sensor   a[3]=back sensor  a[4]=right boundary
    if (a[0] == 1 && a[1] == 1 && fs == 0 && a[2] == 1 && a[3] == 1 && a[4] == 1 && a[5] == 1 && a[6] == 1 )
    {
      Serial.println("hurrah.....we did it!!!!");
      break;
    }
//    analogWrite(in1_R, LOW); //OVERSHOOT
//    analogWrite(in2_R, LOW);
//    analogWrite(in1_L, LOW);
//    analogWrite(in2_L, LOW);
  }
}

int printgraph(graph* g)
{
  int k;
  for (k = 0; k <= g->v; ++k)
  {
    Serial.println("\n Adjacency list of vertex \n head ");
    while (k <= v)
    { 
      Serial.print(" lets start printing ");
      Serial.print(g->arr[k]->head->count1);
      Serial.print( g->arr[k]->head->count2);
      Serial.print( g->arr[k]->head->count3);
      Serial.print( g->arr[k]->head->r);
      Serial.print( g->arr[k]->head->distx);
      Serial.println( g->arr[k]->head->disty);
      break;
    }
    Serial.println("\n");
  }
  return;
}

void setup()
{
  Serial.begin(9600);
  pinMode(in1_R, OUTPUT);
  pinMode(in2_R, OUTPUT);
  pinMode(in1_L, OUTPUT);
  pinMode(in2_L, OUTPUT);
  pinMode(front_sensor, INPUT);
  pinMode(35, OUTPUT);
  digitalWrite(35, HIGH); // calibration pin of ir
  for (int i = 0; i < 7; i++)
    pinMode(IRpins[i], INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  //pinMode(trigPin, OUTPUT);
  //pinMode(echoPin, INPUT);
  delay(2000);
}

void loop()
{
  graph* g = NULL;
  Serial.println("Creating Graph");
  g = creategraph(v);
  start(g, count1, count2, count3);
  printgraph(g);
  //    if(but=HIGH)
  //    {for(k=0;k<=v;k++)
          
  //      if(//sensor condition)
  //     {
  //      pid();
  //    
  //   
  //
  //      if(g->arr[k]->head!=NULL)
  //      {
  //        int a=g->arr[k]->count1;
  //        int b=g->arr[k]->count2;
  //        int c=g->arr[k]->count3;
  //        if()
  //        {
  //          case a://turn left
  //          case b: // move straight
  //          case c: //turn right
  //
  //        }
  //      }
  //    }
  //      }
}


