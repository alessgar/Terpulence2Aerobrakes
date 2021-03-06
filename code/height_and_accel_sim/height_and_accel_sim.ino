float HEIGHT(float time) {
  // Test code to get height as a function of time
float h;

h=-5.712028063268500e-18*pow(time,20) +1.58638919134085e-15*pow(time,19)  +-2.03963339274347e-13*pow(time,18)   +1.61097279401726e-11*pow(time,17)  -8.74654469840995e-10*pow(time,16)
  +3.46070057741233e-08*pow(time,15)  -1.03211061692725e-06*pow(time,14) +2.36677401044779e-05*pow(time,13)    -0.000421810515211216*pow(time,12)   +0.00586438366049921*pow(time,11)
  -0.0634379304342509*pow(time,10)    +0.529037766547874*pow(time,9)      -3.34250566958262*pow(time,8)        +15.5422950299675*pow(time,7)         -50.7101186367172*pow(time,6)
  +106.864381898660*pow(time,5)       -123.313329687538*pow(time,4)      +45.3049941055702*pow(time,3)         +53.4638033298709*pow(time,2)        +4.11040674672448*pow(time,1)
  -0.417857259296764;
}

float ACCEL(float time) {
  // Test code to get accel as a function of time
float a;

a=1.17566104537301e-16*pow(time,20) -1.97662506733408e-14*pow(time,19)   +1.37485165115879e-12*pow(time,18)    -4.49097198565159e-11*pow(time,17)   +1.11439571036918e-10*pow(time,16)
  +5.50907929567516e-08*pow(time,15)  -2.71500418887854e-06*pow(time,14) +7.52815104986264e-05*pow(time,13)    -0.00142742653043154*pow(time,12)    +0.0196098570656512*pow(time,11)
  -0.198888899325335*pow(time,10)    +1.48685709336237*pow(time,9)       -8.03363261114509*pow(time,8)         +29.9576482902292*pow(time,7)        -69.1211945242324*pow(time,6)
  +64.3759633958567*pow(time,5)       +106.004289556379*pow(time,4)      -374.110145583987*pow(time,3)         +412.600835330797*pow(time,2)        -185.671454730500*pow(time,1)
  -80.6565766606938;
}
