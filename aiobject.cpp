#include "aiobject.h"

AIObject::AIObject(int start_x, int start_y, int arm_length)
{
    arm_len = arm_length;
    cur_x = start_x;
    cur_y = start_y;
    dest_x = 400;
    dest_y = 400;
    kp = 5;     //proportional gain
    ki = 3;   //integral gain
    kd = 0.05;  //derivative gain
    da = 0.01;     //simulation angle
    angle = 0.0;
    integral = 0.0;
    prevError = 0.0;
    pop_environment(); // Initial environment is a single pole with coordinates (100,100), (110,100), (110,110), (100,110);

}

void AIObject::pop_environment()
{
    //Populate environment 1 for occupied, 0 for unoccupied
    for(int i = 0; i < ENVSIZE; i++){
        for(int y = 0; y < ENVSIZE; y++){
            //if(i == 100  && y ==100){
              //  environment[i][y] = 1;
            //}
             environment[i][y] = 0;
        }
    }
}

QList<int> AIObject::scan(int x, int y, QString arm_dir) // arm_dir --> N,E,S,W
{
    QList<int> retValues;
    QString x_dir, y_dir;
    int desired_x = cur_x, desired_y = cur_y;
    int displacement = 999;
    int obs_distance = -1;
    int new_arm = ceil(float(arm_len)/10.0);
    int x_val, y_val;
    bool corner_found = false;

    //Figure direction
    if(cur_x > dest_x) x_dir = "West";
    else if(cur_x < dest_x) x_dir = "East";
    else x_dir = "Set";

    if(cur_y > dest_y) y_dir = "South";
    else if(cur_y < dest_y) y_dir = "North";
    else y_dir = "Set";
    
    // N, E, S, W -- (x,y) locations of proximity sensors
    int x_array[4] = {x, (x+new_arm), x, (x-new_arm)};
    int y_array[4] = {(y+new_arm), y, (y-new_arm), y};

    if(arm_dir == "N") {
        x_val = x_array[0];
        y_val = y_array[0];
        // Will iterate over 93 -- (x,y) pairs to find the closest occupancy (if any)
        for(int x = 4; x >= -4; x--) {
            for(int y = 15; y >= 12; y--) {
                if(environment[x_val + x][y_val + y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val+y)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val+x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val + y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is an eastern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x - 1][y_val + y]){
                        if(!corner_found || x_dir == "West") {
                            desired_x = x_val + x - (1 + new_arm);
                            desired_y = y_val + y;
                        }
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 3; x >= -3; x--) {
            for(int y = 11; y >= 8; y--) {
                if(environment[x_val + x][y_val + y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val+y)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val+x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val + y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is an eastern side!";
                    }
                    if(!environment[x_val + x - 1][y_val + y]){
                        desired_x = x_val + x - (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 2; x >= -2; x--) {
            for(int y = 7; y >= 4; y--) {
                if(environment[x_val + x][y_val + y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val+y)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val+x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val + y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is an eastern side!";
                    }
                    if(!environment[x_val + x - 1][y_val + y]){
                        desired_x = x_val + x - (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 1; x >= -1; x--) {
            for(int y = 3; y >= 1; y--) {
                if(environment[x_val + x][y_val + y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val+y)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val+x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val + y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is an eastern side!";
                    }
                    if(!environment[x_val + x - 1][y_val + y]){
                        desired_x = x_val + x - (1 + new_arm);
                        desired_y = y_val + y;
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int y = 15; y >= 1; y--) {
            if(environment[x_val][y_val + y] || (x_val>=ENVSIZE) || (x_val<=0) || ((y_val+y)>=ENVSIZE)){
                qDebug() <<  "(" << x_val << "," << y_val+y << ") is occupied";
                // Check for corners //
                if(!environment[x_val + 1][y_val + y]) {
                    desired_x = x_val + (1 + new_arm);
                    desired_y = y_val + y;
                    qDebug() << "(" << x_val << "," << y_val + y << ") is an eastern side!";
                }
                if(!environment[x_val - 1][y_val + y]){
                    desired_x = x_val - (1 + new_arm);
                    desired_y = y_val + y;
                    qDebug() << "(" << x_val << "," << y_val + y << ") is a western side!";
                }
                ///////////////////////
                if(obs_distance >= 0) {
                    if(y < obs_distance) obs_distance = y;
                }
                else obs_distance = y;
            }
        }

        displacement = desired_x - cur_x;

    }
    else if(arm_dir == "E"){
        x_val = x_array[1];
        y_val = y_array[1];

        // Will iterate over 93 -- (x,y) pairs to find the closest occupancy (if any)
        for(int y = 4; y >= -4; y--) {
            for(int x = 15; x >= 12; x--) {
                if(environment[x_val + x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val+x)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val + x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x][y_val + y + 1]) {
                        desired_x = x_val + x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val + x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 3; y >= -3; y--) {
            for(int x = 15; x >= 8; x--) {
                if(environment[x_val + x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val+x)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val + x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x][y_val + y + 1]) {
                        desired_x = x_val + x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val + x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 2; y >= -2; y--) {
            for(int x = 15; x >= 4; x--) {
                if(environment[x_val + x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val+x)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val + x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x][y_val + y + 1]) {
                        desired_x = x_val + x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val + x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 1; y >= -1; y--) {
            for(int x = 15; x >= 1; x--) {
                if(environment[x_val + x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val+x)>=ENVSIZE)){
                    qDebug() <<  "(" << x_val + x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x][y_val + y + 1]) {
                        desired_x = x_val + x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val + x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val + x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int x = 15; x >= 1; x--) {
            if(environment[x_val + x][y_val] || (y_val>=ENVSIZE) || (y_val<=0) || ((x_val+x)>=ENVSIZE)){
                qDebug() <<  "(" << x_val + x << "," << y_val << ") is occupied";
                // Check for corners //
                if(!environment[x_val + x][y_val + 1]) {
                    desired_x = x_val + x;
                    desired_y = y_val + (1 + new_arm);
                    qDebug() << "(" << x_val + x << "," << y_val << ") is a northern side!";
                    corner_found = true;
                }
                if(!environment[x_val + x][y_val - 1]){
                    if(!corner_found || y_dir == "South") {
                        desired_x = x_val + x;
                        desired_y = y_val - new_arm;
                    }
                    qDebug() << "(" << x_val + x << "," << y_val << ") is a southern side!";
                }
                ///////////////////////
                if(obs_distance >= 0) {
                    if(x < obs_distance) obs_distance = x;
                }
                else obs_distance = x;
            }
        }

        displacement = desired_y - cur_y;

    }
    else if(arm_dir == "S") {
        x_val = x_array[2];
        y_val = y_array[2];

        // Will iterate over 93 -- (x,y) pairs to find the closest occupancy (if any)
        for(int x = 4; x >= -4; x--) {
            for(int y = 15; y >= 12; y--) {
                if(environment[x_val + x][y_val - y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val-y)<=0)){
                    qDebug() <<  "(" << x_val+x << "," << y_val-y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val - y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val - y;
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is an eastern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x - 1][y_val - y]){
                        if(!corner_found || x_dir == "West") {
                            desired_x = x_val + x - (1 + new_arm);
                            desired_y = y_val - y;
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 3; x >= -3; x--) {
            for(int y = 15; y >= 8; y--) {
                if(environment[x_val + x][y_val - y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val-y)<=0)){
                    qDebug() <<  "(" << x_val+x << "," << y_val-y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val - y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val - y;
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is an eastern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x - 1][y_val - y]){
                        if(!corner_found || x_dir == "West") {
                            desired_x = x_val + x - (1 + new_arm);
                            desired_y = y_val - y;
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 2; x >= -2; x--) {
            for(int y = 15; y >= 4; y--) {
                if(environment[x_val + x][y_val - y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val-y)<=0)){
                    qDebug() <<  "(" << x_val+x << "," << y_val-y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val - y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val - y;
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is an eastern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x - 1][y_val - y]){
                        if(!corner_found || x_dir == "West") {
                            desired_x = x_val + x - (1 + new_arm);
                            desired_y = y_val - y;
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int x = 1; x >= -1; x--) {
            for(int y = 15; y >= 1; y--) {
                if(environment[x_val + x][y_val - y] || ((x_val+x)>=ENVSIZE) || ((x_val+x)<=0) || ((y_val-y)<=0)){
                    qDebug() <<  "(" << x_val+x << "," << y_val-y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val + x + 1][y_val - y]) {
                        desired_x = x_val + x + (1 + new_arm);
                        desired_y = y_val - y;
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is an eastern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val + x - 1][y_val - y]){
                        if(!corner_found || x_dir == "West") {
                            desired_x = x_val + x - (1 + new_arm);
                            desired_y = y_val - y;
                        }
                        qDebug() << "(" << x_val + x << "," << y_val - y << ") is a western side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(y < obs_distance) obs_distance = y;
                    }
                    else obs_distance = y;
                }
            }
        }

        for(int y = 15; y >= 1; y--) {
            if(environment[x_val][y_val - y] || (x_val>=ENVSIZE) || (x_val<=0) || ((y_val-y)<=0)){
                qDebug() <<  "(" << x_val << "," << y_val-y << ") is occupied";
                // Check for corners //
                if(!environment[x_val + 1][y_val - y]) {
                    desired_x = x_val + (1 + new_arm);
                    desired_y = y_val - y;
                    qDebug() << "(" << x_val << "," << y_val - y << ") is an eastern side!";
                    corner_found = true;
                }
                if(!environment[x_val - 1][y_val - y]){
                    if(!corner_found || x_dir == "West") {
                        desired_x = x_val - (1 + new_arm);
                        desired_y = y_val - y;
                    }
                    qDebug() << "(" << x_val << "," << y_val - y << ") is a western side!";
                }
                ///////////////////////
                if(obs_distance >= 0) {
                    if(y < obs_distance) obs_distance = y;
                }
                else obs_distance = y;
            }
        }

        displacement = desired_x - cur_x;

    }
    else if(arm_dir == "W"){
        x_val = x_array[3];
        y_val = y_array[3];

        // Will iterate over 93 -- (x,y) pairs to find the closest occupancy (if any)
        for(int y = 4; y >= -4; y--) {
            for(int x = 15; x >= 12; x--) {
                if(environment[x_val - x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val-x)<=0)){
                    qDebug() <<  "(" << x_val - x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val - x][y_val + y + 1]) {
                        desired_x = x_val - x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val - x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val - x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val - x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val - x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 3; y >= -3; y--) {
            for(int x = 11; x >= 8; x--) {
                if(environment[x_val - x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val-x)<=0)){
                    qDebug() <<  "(" << x_val - x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val - x][y_val + y + 1]) {
                        desired_x = x_val - x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val - x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val - x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val - x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val - x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 2; y >= -2; y--) {
            for(int x = 7; x >= 4; x--) {
                if(environment[x_val - x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val-x)<=0)){
                    qDebug() <<  "(" << x_val - x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val - x][y_val + y + 1]) {
                        desired_x = x_val - x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val - x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val - x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val - x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val - x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int y = 1; y >= -1; y--) {
            for(int x = 3; x >= 1; x--) {
                if(environment[x_val - x][y_val + y] || ((y_val+y)>=ENVSIZE) || ((y_val+y)<=0) || ((x_val-x)<=0)){
                    qDebug() <<  "(" << x_val - x << "," << y_val+y << ") is occupied";
                    // Check for corners //
                    if(!environment[x_val - x][y_val + y + 1]) {
                        desired_x = x_val - x;
                        desired_y = y_val + y + (1 + new_arm);
                        qDebug() << "(" << x_val - x << "," << y_val + y << ") is a northern side!";
                        corner_found = true;
                    }
                    if(!environment[x_val - x][y_val + y - 1]){
                        if(!corner_found || y_dir == "South") {
                            desired_x = x_val - x;
                            desired_y = y_val - (y + new_arm);
                        }
                        qDebug() << "(" << x_val - x << "," << y_val - y << ") is a southern side!";
                    }
                    ///////////////////////
                    if(obs_distance >= 0) {
                        if(x < obs_distance) obs_distance = x;
                    }
                    else obs_distance = x;
                }
            }
        }

        for(int x = 15; x >= 1; x--) {
            if(environment[x_val - x][y_val] || ((y_val)>=ENVSIZE) || ((y_val)<=0) || ((x_val-x)<=0)){
                qDebug() <<  "(" << x_val - x << "," << y_val << ") is occupied";
                // Check for corners //
                if(!environment[x_val - x][y_val + 1]) {
                    desired_x = x_val - x;
                    desired_y = y_val + (1 + new_arm);
                    qDebug() << "(" << x_val - x << "," << y_val << ") is a northern side!";
                    corner_found = true;
                }
                if(!environment[x_val - x][y_val - 1]){
                    if(!corner_found || y_dir == "South") {
                        desired_x = x_val - x;
                        desired_y = y_val - new_arm;
                    }
                    qDebug() << "(" << x_val - x << "," << y_val << ") is a southern side!";
                }
                ///////////////////////
                if(obs_distance >= 0) {
                    if(x < obs_distance) obs_distance = x;
                }
                else obs_distance = x;
            }
        }

        displacement = desired_y - cur_y;
    }

    if(environment[x_val][y_val] || (x_val>=ENVSIZE) || (x_val<=0) || (y_val>=ENVSIZE) || (y_val<=0)) obs_distance = 0;
    qDebug() << y_dir << x_dir;
    qDebug() << "(" << desired_x << "," << desired_y << ")";
    retValues << obs_distance << displacement;
    return retValues;
}

bool AIObject::setDestination(int x, int y)
{
    int new_arm = ceil(arm_len);
    if(x > 0+new_arm && x < ENVSIZE-new_arm && y > 0+new_arm && y < ENVSIZE-new_arm) {
        dest_x = x;
        dest_y = y;
        return true;
    }
    else return false;
}

void AIObject::getTargetAngles(double& pitch, double& roll, int x, int y)
{

    QList<int> distances, displacements;
    QString x_dir, y_dir;
    cur_x = x;
    cur_y = y;

    //Figure direction
    if(cur_x > dest_x) x_dir = "West";
    else if(cur_x < dest_x) x_dir = "East";
    else x_dir = "Set";

    if(cur_y > dest_y) y_dir = "South";
    else if(cur_y < dest_y) y_dir = "North";
    else y_dir = "Set";

    QList<int> N, E, S, W;
    N = scan(cur_x, cur_y, "N"); //distance, displacement
    E = scan(cur_x, cur_y, "E");
    S = scan(cur_x, cur_y, "S");
    W = scan(cur_x, cur_y, "W");

    distances << N.at(0) << E.at(0) << S.at(0) << W.at(0);
    displacements << N.at(1) << E.at(1) << S.at(1) << W.at(1);

    if(x_dir == "North") { //Need to head North!
        if(y_dir == "East") { //Need to head East!
            if(distances.at(0) < 0 && distances.at(1) < 0) { // NorthEast is free of obstacles
                pitch = angleController(dest_y);
                roll = angleController(dest_x);
            }
            else if(distances.at(0) < 0 && distances.at(1) >= 0) { // North is free but East has obstacles
                pitch = angleController(dest_y);
                roll = angleController(displacements.at(1));
            }
            else if(distances.at(0) >= 0 && distances.at(1) < 0) { //North is blocked but East is free
                pitch = angleController(displacements.at(0));
                roll = angleController(dest_x);
            }
            else { // NorthEast is blocked
                pitch = angleController(displacements.at(0));
                roll = angleController(displacements.at(1));
            }
        }
        else if(y_dir == "West"){ //Need to head West!
            if(distances.at(0) < 0 && distances.at(3) < 0) { //No obstacle
                pitch = angleController(dest_y);
                roll = angleController(dest_x);
            }
            else if(distances.at(0) < 0 && distances.at(3) >= 0) { // North is free but West has obstacles
                pitch = angleController(dest_y);
                roll = angleController(displacements.at(3));
            }
            else if(distances.at(0) >= 0 && distances.at(3) < 0) { //North is blocked but East is free
                pitch = angleController(displacements.at(0));
                roll = angleController(dest_x);
            }
            else { // NorthEast is blocked
                pitch = angleController(displacements.at(0));
                roll = angleController(displacements.at(3));
            }
        }
        else{
            if(distances.at(0) < 0) {
                pitch = angleController(dest_y);
                roll = DEFAULT_ROLL;
            }
            else {
                pitch = angleController(displacements.at(0));
                roll = DEFAULT_ROLL;
            }
        }
    }
    else if(x_dir == "South") { //Need to head South!
        if(y_dir == "East") { //Need to head East!
            if(distances.at(2) < 0 && distances.at(1) < 0) { // SouthEast is free of obstacles
                pitch = angleController(dest_y);
                roll = angleController(dest_x);
            }
            else if(distances.at(2) < 0 && distances.at(1) >= 0) { // South is free but East has obstacles
                pitch = angleController(dest_y);
                roll = angleController(displacements.at(1));
            }
            else if(distances.at(2) >= 0 && distances.at(1) < 0) { //South is blocked but East is free
                pitch = angleController(displacements.at(2));
                roll = angleController(dest_x);
            }
            else { // SouthEast is blocked
                pitch = angleController(displacements.at(2));
                roll = angleController(displacements.at(1));
            }
        }
        else if(y_dir == "West"){ //Need to head West!
            if(distances.at(2) < 0 && distances.at(3) < 0) { //No obstacle
                pitch = angleController(dest_y);
                roll = angleController(dest_x);
            }
            else if(distances.at(2) < 0 && distances.at(3) >= 0) { // South is free but West has obstacles
                pitch = angleController(dest_y);
                roll = angleController(displacements.at(3));
            }
            else if(distances.at(2) >= 0 && distances.at(3) < 0) { //South is blocked but East is free
                pitch = angleController(displacements.at(2));
                roll = angleController(dest_x);
            }
            else { // SouthEast is blocked
                pitch = angleController(displacements.at(2));
                roll = angleController(displacements.at(3));
            }
        }
        else{
            if(distances.at(2) < 0) {
                pitch = angleController(dest_y);
                roll = DEFAULT_ROLL;
            }
            else {
                pitch = angleController(displacements.at(2));
                roll = DEFAULT_ROLL;
            }
        }
    }
    else {
        if(y_dir == "East") {
            if(distances.at(1) < 0) {
                pitch = DEFAULT_PITCH;
                roll = angleController(x_dest);
            }
            else {
                pitch = DEFAULT_PITCH;
                roll = angleController(displacements.at(1));
            }
        }
        else if(y_dir == "West"){
            if(distances.at(3) < 0) {
                pitch = DEFAULT_PITCH;
                roll = angleController(x_dest);
            }
            else {
                pitch = DEFAULT_PITCH;
                roll = angleController(displacements.at(3));
            }
        }
        else { //Destination reached!
            qDebug() << "Destination Reached!";
            pitch = DEFAULT_PITCH;
            roll = DEFAULT_ROLL;
        }
    }

}

float AIObject::angleController(int destination)
{
    float angle = 0.0;
    float error = destination;
    float derivative;
    if(abs(error) > 0.01) {
        integral += (error+prevError)*da;
        if( integral > 100) {
            integral = 100;
        }
        if( integral < 0 ) {
            integral = 0;
        }
    }
    derivative = (error - prevError) / da;
    angle = error*kp + integral*ki + derivative*kd;

    if( angle > 30 ) {
        angle = 30;
    }
    if( angle < 0 ) {
        angle = 0.001;
    }

    prevError = error;

    return angle;
}
