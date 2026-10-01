#include "Eskf9.h"
#include "MotionModelFF.h"
#include <cstdio>
int main(){
  Eskf9 e;
  float a[3]={0,0,9.80665f}, g[3]={0,0,0};
  for(int i=0;i<100;i++){e.predict(a,g,0.004f);e.updateGravityDirection(a,0.04f);}
  MotionModelFeedforward m;
  auto o=m.update(0,0,0,0,0.004f,505);
  for(int i=0;i<100;i++) o=m.update(5,3,0,0,0.004f,505);
  std::printf("healthy=%d z=%f valid=%d m1=%f m2=%f\n",e.healthy(),e.positionZ(),o.valid,o.motorDeltaDshot[0],o.motorDeltaDshot[1]);
}
