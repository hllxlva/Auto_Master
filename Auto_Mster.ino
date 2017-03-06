float data[4] = {0,0.1,0.2,0.1};//エンコーダの値

unsigned long time;
unsigned long t0;
float dt;

//Approximate-------------------------------
int initial_value[] = {0,0,0};//初期位置
float now_p[3][8];//今の位置 0:X, 1:Y, 2:Ang
//int now_p_int[3];//今の位置の平均 0:X, 1:Y, 2:Ang
float now_p_ave[3];//今の位置の平均 0:X, 1:Y, 2:Ang[rad]//------------------
float now_v[3][4];//今の速度 0:X, 1:Y, 2:Ang
//float dif_v[3];//速度の違い 0:X, 1:Y, 2:Ang
//float dif_p[3];//位置の違い 0:X, 1:Y, 2:Ang
int n = 1;//初期設定フラグ
float Cr[4];//センサーまでの距離

//route_trace-------------------------------
#include "route_saka_back.h"//ROUTE_POINT_NUM, signed short route[][]
boolean route_read = true;
boolean flag, PRE_R = true;
//int[][] data;
//float now_p_ave[3] = {0,0,90};//now_p_ave[3]に変更した
float min_m_dist;
float pre_min_m_dist = 1000;
int min_m_dist_num;
float v[3];//演算した速度{x, y, θ}
float v_t[2];//接線方向の速度
//float v_n[2];//法線方向の速度
float p_v[2];//比例制御
//float d_v[2];//微分制御
float r;//
float pre_r;
//float vx,vy,vx_t,vy_t,vx_n,vy_n;//接線方向と法線方向の速度
float e, eq, pre_pos;//, pre_e, pre_eq;//編差
float v_max[] = {10,10};//最高速度{並進, 回転}
float slow_stop = 1.0;//slow_stop以外のところでの倍率
float slow_start = 1.0;//slow_start以外のところでの倍率
float slow = 5;//スローで何％まで落とすか
//float Cr[4][2];//Censorの極座標表示[0] = x,[1] = y
float M[4][3] = {{100, -100, 45}, //Mecanumの位置{x, y, メカナムの角度}
               {-100, -100, 135}, 
               {-100, 100, 225},
               {100, 100, 315}};
//float Mr[4][2];//Mecanuumの極座標表示[0] = x,[1] = y
float Kt = 20/*20*/, Kp = 2/*5*/;//, Kd = 2;//2
float Cp = 0.1, Cd = 0.5;
float V_rotation[4][2];
float V_translation[4][2];
float V_resultant[4][2];
float V_out_float[4];
int V_out[4];
float V_out_max=255;


void setup() {
  Serial.begin(250000);
}

void Approx(float Vd[4]){//センサーの位置，エンコーダーの値から自己位置を推定する．
  int C[4][2] = {//センサーの位置
    { 300, 0},
    { 0, 300},
    {-300, 0},
    { 0,-300}
  };
  if(n == 1){//初期設定
    float Cp[4][2];
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 2; j++) {
        Cp[i][j] = float(C[i][j])/100;//値が大きすぎてintやと足りない…
      }
    }
    for (int i = 0; i < 4; i++) {
      Cr[i] = 100*sqrt(Cp[i][0]*Cp[i][0]+Cp[i][1]*Cp[i][1]);//極座標系に
    }
    for (int i = 0; i < 3; i++){
      for (int j = 0; j < 8; j++){
        now_p[i][j] = initial_value[i];//初期値
      }
    }
    n = 0;
  }
  //float w[2];//ω
  for(int i = 0; i < 2; i++){//ω４つのセンサーから２通りωが出せる．
    now_v[2][i] = (Vd[i]+Vd[i+2])/(Cr[i]+Cr[i+2]);
  }
  //dif_v[2] = abs(now_v[2][0]-now_v[2][1]);
  //if(dif_v < 1){
    for (int i = 0; i < 2; i++){
      for (int j = 0; j < 2; j++){//速度の各成分は４通りの出し方がある
        now_v[0][2*i+j] = (-2*i+1)*(Cr[2*i+1]*now_v[2][j]-Vd[2*i+1]);
        now_v[1][2*i+j] = (-2*i+1)*(Vd[2*i]-Cr[2*i]*now_v[2][j]);
      }
    }
    for (int i = 0; i < 2; i++){//θを求める
      now_p[2][i] = now_p[2][i] + now_v[2][i];
      //if(now_p[2][i] > 2*PI)now_p[2][i] - 2*PI;
    }
    //dif_p[2] = abs(now_p[2][0]-now_p[2][1]);
    now_p_ave[2] = (now_p[2][0]+now_p[2][1])/2;
    now_p_ave[2] = now_p_ave[2]*180/PI;
    for (int i = 0; i < 4; i++){//位置の各成分はここの４通りと
      now_p[0][i] = now_p[0][i]+(now_v[0][i]*cos(now_p[2][0])-now_v[1][i]*sin(now_p[2][0]));
      now_p[1][i] = now_p[1][i]+(now_v[0][i]*sin(now_p[2][0])+now_v[1][i]*cos(now_p[2][0]));
    }
    for (int i = 0; i < 4; i++){//ここの４通りで求めることが出来る．
      now_p[0][i+4] = now_p[0][i+4]+(now_v[0][i]*cos(now_p[2][1])-now_v[1][i]*sin(now_p[2][1]));
      now_p[1][i+4] = now_p[1][i+4]+(now_v[0][i]*sin(now_p[2][1])+now_v[1][i]*cos(now_p[2][1]));
    }
    for (int i = 0; i < 2; i++){
      now_p_ave[i] = 0;
      for (int j = 0; j < 8; j++){
        now_p_ave[i] = (j*now_p_ave[i]+now_p[i][j])/(j+1);//重心を求める
      }
    }
    /*for (int i = 0; i < 3; i++){
      if (i == 2)now_p_int[i] = 180/PI*(now_p_ave[i])*10;//精度欲しいから10倍しとく
      else now_p_int[i] = now_p_ave[i];//0:X, 1:Y, 2:Ang をint型に直す．
    }*/
  //}
}

void velocity(){
  for(int i = 0; i < ROUTE_POINT_NUM; i++){//マンハッタン距離最小値
    min_m_dist = sq(now_p_ave[0]-route[i][0])+sq(now_p_ave[1]-route[i][1]);
    if(flag){
      pre_min_m_dist = min_m_dist;
      flag = false;
    }
    if(pre_min_m_dist >= min_m_dist){
      pre_min_m_dist = min_m_dist;
      min_m_dist_num = i;
    }
  }
  flag = true;
  if(min_m_dist_num == ROUTE_POINT_NUM - 1){//最後まで行ったら速度0に
    //接線方向の速度
    v_t[0] = route[min_m_dist_num][0] - now_p_ave[0];
    v_t[1] = route[min_m_dist_num][1] - now_p_ave[1];
    r = sqrt(sq(v_t[0])+sq(v_t[1]));
    if(PRE_R){
      pre_r = r;
      if(pre_r < 1)pre_r = 1;
      PRE_R = false;
    }
    v_t[0] = v_t[0]/pre_r;
    v_t[1] = v_t[1]/pre_r;
    //法線方向の偏差
    e = 0;
    p_v[0] = 0;
    p_v[1] = 0;
    //d_v[0] = 0;//----
    //d_v[1] = 0;//----
  }else{
    PRE_R = true;
    //接線方向の速度
    v_t[0] = route[min_m_dist_num + 1][0] - route[min_m_dist_num][0];
    v_t[1] = route[min_m_dist_num + 1][1] - route[min_m_dist_num][1];
    r = sqrt(sq(v_t[0])+sq(v_t[1]));//大きさを計算
    v_t[0] = v_t[0]/r;//方向のみ
    v_t[1] = v_t[1]/r;
    //法線方向の偏差
    //pre_e = e;
    e = sqrt(sq((route[min_m_dist_num][1] - now_p_ave[1])*(v_t[0])+(now_p_ave[0] - route[min_m_dist_num][0])*(v_t[1]))/(sq(v_t[0])+sq(v_t[1])));
    //線のどちら側にあるかを調べる
    if(v_t[0]*(route[min_m_dist_num][1] - now_p_ave[1])+v_t[1]*(now_p_ave[0] - route[min_m_dist_num][0]) > 0){
      e = -e;
    }
    //法線方向の比例制御
    p_v[0] = e * +v_t[1]/r;
    p_v[1] = e * -v_t[0]/r;
    //法線方向の微分制御
    //d_v[0] = (pre_e - e) * +v_t[1]/r;//----
    //d_v[1] = (pre_e - e) * -v_t[0]/r;//----
  }
  //法線方向の速度使わないと決めた
  //v_n[0] = route[min_m_dist_num][0] - now_p_ave[0];
  //v_n[1] = route[min_m_dist_num][1] - now_p_ave[1];
  
  //制御の係数を代入
  v[0] = v_t[0]*Kt + p_v[0]*Kp;// + d_v[0]*Kd;//----
  v[1] = v_t[1]*Kt + p_v[1]*Kp;// + d_v[1]*Kd;//----
  float R = sq(v[0]) + sq(v[1]);
  if(R>sq(v_max[0])){//最高速度
    v[0] = v_max[0]*v[0]/sqrt(R);
    v[1] = v_max[0]*v[1]/sqrt(R);
  }
  //------------角度操作---------------------
  //pre_eq = eq;
  eq = route[min_m_dist_num][2] - now_p_ave[2];
  v[2] = eq * Cp + (now_p_ave[2] - pre_pos) * Cd;//v[2] = eq * Cp - (pre_pos - now_p_ave[2]) * Cd;
  if(v[2] > v_max[1])v[2] = v_max[1];
  if(v[2] < -v_max[1])v[2] = -v_max[1];
  pre_pos = now_p_ave[2];
  
  //回転方向速度ベクトル
  for (int j = 0; j < 4; j++) {
    for (int i = 0; i < 2; i++){
      V_rotation[j][i] = (-1+2*i)*v[2]*PI/180*M[j][1-1*i];
    }
  }
  //並進方向速度ベクトル
  for (int i = 0; i < 4; i++) {
    V_translation[i][0] =  v[0]*cos(now_p_ave[2]*PI/180)+v[1]*-sin(now_p_ave[2]*PI/180);
    V_translation[i][1] =  v[0]*sin(now_p_ave[2]*PI/180)+v[1]*cos(now_p_ave[2]*PI/180);
  }
  //並進回転合成ベクトル
  for(int i = 0; i < 4; i++){
    for(int j = 0; j < 2; j++){
      V_resultant[i][j] = -V_rotation[i][j]+V_translation[i][j];
    }
  }
  //各メカナム出力
  for (int i = 0; i < 4; i++) {
    V_out_float[i] = V_resultant[i][0]*sin((M[i][2])*PI/180)+V_resultant[i][1]*cos((M[i][2])*PI/180);
  }
  //最高速度設定
  boolean flag_v = true;
  float unsign_v, pre_unsign_v;
  int max_v_num = 0;
  for(int i = 0; i < 4; i++){//最高速度
    unsign_v = abs(V_out_float[i]);
    if(flag_v){
      pre_unsign_v = unsign_v;
      flag_v = false;
    }
    if(pre_unsign_v <= unsign_v){
      pre_unsign_v = unsign_v;
      max_v_num = i;
    }
  }
  
  //スローストップ
  int slow_stop_count = ROUTE_POINT_NUM-1-min_m_dist_num;
  if(slow_stop_count <= 20){//最後から何個前の点から減速するか
    slow_stop = slow_stop_count/20.0*(1-slow/100) + slow/100;
    if(slow_stop > 1.0) slow_stop = 1.0;
  }else{
    slow_stop = 1.0;
  }

  //スロースタート
  int slow_start_count = min_m_dist_num;
  if(slow_start_count <= 20){//最初から何個後の点まで減速するか
    slow_start = slow_start_count/20.0*(1-slow/100) + slow/100;
    if(slow_start > 1.0) slow_start = 1.0;
  }else{
    slow_start = 1.0;
  }

  //最高速度を最高出力に合わせる
  float V_max = abs(V_out_float[max_v_num]);
  for (int i = 0; i < 4; i++){
    V_out_float[i] = V_out_float[i] / V_max * V_out_max * slow_stop * slow_start;
  }
  for (int i = 0; i < 4; i++){
    if(V_out_float[i] > 0){
      V_out[i] = V_out_float[i]+0.5;
    }else{
      V_out[i] = abs(V_out_float[i])+0.5;
      V_out[i] = -V_out[i];
    }
  }
}

void loop() {
  time = micros();
  dt = float(time - t0)/1000;//プログラムの周期ms
  t0 = time;
  Approx(data);
  velocity();//v[0], v[1], v[2]を出す, それぞれのメカナムの出力を返す
  for (int i = 0; i < 2; i++) {
    Serial.print(route[min_m_dist_num][i]);
    Serial.print("||");
  }
  for (int i = 0; i < 3; i++) {
    Serial.print(now_p_ave[i]);
    Serial.print("|");
  }
  for (int i = 0; i < 4; i++) {
    Serial.print(V_out_float[i]);
    Serial.print("|");
    Serial.print(V_out[i]);
    Serial.print("|");
  }
  Serial.println(dt);
}
