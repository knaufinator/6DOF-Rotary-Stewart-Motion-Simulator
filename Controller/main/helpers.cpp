#include "helpers.h"

// Helper function to map float values
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Rate limiting function to smooth transitions
float rateLimit(float target, float current) {
    float diff = target - current;
    float maxChange = 0.01; // Maximum change allowed per update
    
    if(diff > maxChange)
        return current + maxChange;
    else if(diff < -maxChange)
        return current - maxChange;
    else
        return target;
}

//function calculating needed servo rotation value
float getAlpha(int i,volatile float arr[]){
            
    // for Platform Coord algorithm
    float platformPDx[6]={0,0,0, 0,0,0};
    float platformPDy[6]={0,0,0, 0,0,0};
    float platformAngle[6]={0,0,0,0,0,0};
    float platformCoordsx[6]={0,0,0, 0,0,0};
    float platformCoordsy[6]={0,0,0, 0,0,0};
    float basePDx[6]={0,0,0, 0,0,0};
    float basePDy[6]={0,0,0, 0,0,0};
    float baseAngle[6]={0,0,0,0,0,0};
    float baseCoordsx[6]={0,0,0, 0,0,0};
    float baseCoordsy[6]={0,0,0, 0,0,0};
    float DxMultiplier[6]={1,1,1, -1,-1,-1};
    float AngleMultiplier[6]={1,-1,1,1,-1,1};
    float OffsetAngle[6]={pi/6,pi/6,-pi/2, -pi/2,pi/6,pi/6};
    
    //base rotation values
    float platformPivotx[6]={0,0,0, 0,0,0};
    float platformPivoty[6]={0,0,0, 0,0,0};
    float platformPivotz[6]={0,0,0, 0,0,0};
    
    float deltaLx[6] = {0,0,0, 0,0,0};
    float deltaLy[6] = {0,0,0, 0,0,0};
    float deltaLz[6] = {0,0,0, 0,0,0};
    float deltaL2Virtual[6] = {0,0,0, 0,0,0};
    
    float l[6] = {0,0,0, 0,0,0};
    float m[6] = {0,0,0, 0,0,0};
    float n[6] = {0,0,0, 0,0,0};
    float alpha[6] = {0,0,0, 0,0,0};
       
    platformPDx[i] = DxMultiplier[i] * RD;
    platformPDy[i] = RD;
    platformAngle[i] = OffsetAngle[i] + AngleMultiplier[i]* radians(theta_r);
    platformCoordsx[i] = platformPDx[i] * cos(platformAngle[i]);
    platformCoordsy[i] = platformPDy[i] * sin(platformAngle[i]);
    
    basePDx[i] = DxMultiplier[i] * PD;
    basePDy[i] = PD;
    baseAngle[i] = OffsetAngle[i] + AngleMultiplier[i]* radians(theta_p);
    baseCoordsx[i] = basePDx[i] * cos(baseAngle[i]);
    baseCoordsy[i] = basePDy[i] * sin(baseAngle[i]);
    
    //Platform pivots
    platformPivotx[i] = platformCoordsx[i]*cos(arr[3])*cos(arr[5])+platformCoordsy[i]*(sin(arr[4])*sin(arr[3])*cos(arr[3])-cos(arr[4])*sin(arr[5]))+arr[0]; 
    platformPivoty[i] = platformCoordsx[i]*cos(arr[4])*sin(arr[5])+platformCoordsy[i]*(cos(arr[3])*cos(arr[5])+sin(arr[3])*sin(arr[4])*sin(arr[5]))+arr[1];
    platformPivotz[i] = -platformCoordsx[i]*sin(arr[3])+platformCoordsy[i]*sin(arr[4])*cos(arr[3])+platformHeight+arr[2];
    
    deltaLx[i] = baseCoordsx[i] - platformPivotx[i];
    deltaLy[i] = baseCoordsy[i] - platformPivoty[i];
    deltaLz[i] = -platformPivotz[i];
    
    deltaL2Virtual[i] = sqrt(pow(deltaLx[i], 2.0) + pow(deltaLy[i], 2.0) +pow(deltaLz[i], 2.0));

    l[i] = pow(deltaL2Virtual[i], 2.0)-(pow(ConnectingArmLengthL2, 2.0)-pow(ServoArmLengthL1, 2.0));
    m[i] = 2*ServoArmLengthL1*(platformPivotz[i]);
    n[i] = 2*ServoArmLengthL1*(cos(theta_s[i]*pi/180)*(platformPivotx[i] -  baseCoordsx[i])+sin(theta_s[i]*pi/180)*(platformPivoty[i]- baseCoordsy[i]));

    return asin(l[i]/(sqrt(pow(m[i], 2.0)+pow(n[i], 2.0))))-atan(n[i]/m[i]);
}
