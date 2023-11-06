/*
LIDAR-IMU OpenGL visualiser

This project consists of 3 Threads. 
Ethernet packet capture and point cloud creation are executed in the GUI thread. 
Opengl visualization jobs are executed on the 2nd Thread. 
Serial communication with Tiny is executed on the 3rd Thread.

Currently it is working on Windows(x86) environment and if it is necessary to 
execute on Linux distributions you should touch; UDP socket and serial library.

OpenGL GLUT library is freeglut and probably work on both Linux and Windows.
*/

#include <Winsock2.h>
#include <iostream>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include "main.h"
#include <omp.h>
#include <math.h>
#include <thread>
//#include <glfw3.h>
#include <GL/glut.h>
#include <vector>

#ifdef   APP_WINDOWS
    #include <tuple>
    #include <windows.h>
    #include <sstream>
    #include "lidar_exhibition/lidar_exhibition/serial.h"
#else
    #include <errno.h>
    #include <fcntl.h> 
    #include <termios.h>
    #include <unistd.h>
#endif

using namespace std;

#pragma comment(lib,"ws2_32.lib")
#pragma warning(disable:4996) 

Channel     point_frame[36000];
int         packidx = 0;
float       previous_azimuth = 0.0;
GLfloat     angleCube = 0.0f;
int         refreshMills = 25;

#define     PI 3.14159265
#define     RADPERDEG 0.0174533

GLWidget    widget;
TinyIMU     myimu;
PointInfo   myinfo;

float times = 1;
int main_w = 0;
int sub_w = 0;

/*! \brief String splitter with delimiter
* TODO Utile alicaz bunu
 */
vector<string> split(string s, string delimiter) {
    size_t pos_start = 0, pos_end, delim_len = delimiter.length();
    string token;
    vector<string> res;

    while ((pos_end = s.find(delimiter, pos_start)) != string::npos) {
        token = s.substr(pos_start, pos_end - pos_start);
        pos_start = pos_end + delim_len;
        res.push_back(token);
    }

    res.push_back(s.substr(pos_start));
    return res;
}

/*! \brief Util converts something :)
 * Currently only converts distance, azimuth and degree to X,Y,Z.
 */
class Util {

public:
    Util() {
        this->x = 0.0;
        this->y = 0.0;
        this->z = 0.0;
    };

    float   laser_angles[16] = { -15, 1, -13, 3, -11, 5, -9, 7, -7, 9, -5, 11, -3, 13, -1, 15 };
    
    float getX(float distance, float azimuth, int laser_id) {
        x = distance * cos(this->laser_angles[laser_id] * (PI / 180)) * sin(azimuth * (PI / 180));
        return x;
    };

    float getY(float distance, float azimuth, int laser_id) {
        y = distance * cos(this->laser_angles[laser_id] * (PI / 180)) * cos(azimuth * (PI / 180));
        return y;
    };

    float getZ(float distance, float azimuth, int laser_id) {
        z = distance * sin(this->laser_angles[laser_id] * (PI / 180));
        return z;
    };

    //usage: tie(x, y, z) = Util->getXYZ(1,1,1);
    std::tuple<float, float, float> getXYZ(float distance, float azimuth, int laser_id){
        return std::make_tuple(getX(distance, azimuth, laser_id), getY(distance, azimuth, laser_id), getZ(distance, azimuth, laser_id));
    }

private:
    float   x, y, z;    //Actually we don't need but i added.
};

/*! \brief UDP packet parser
 * Used for received packet parsing.
 *
 * Also gives detailed point informations in 
 * getDataBlocks getter.
 */
class Parseudp {


public:
    int laser_id = 0;
    int z = 0;
    int real_laser = 0;

    Parseudp() {
        new_azimuth = 0.0;
        this->laser_id = 0;
        this->z = 0;
        real_laser = 0;
    }

    void setUDP(uint8_t* bufaddr) {
        
        for (int i = 0; i < MAX_NUMS; i++) {
            this->z = 0; 
            this->laser_id = 0;

            try{
                new_azimuth = (float)(bufaddr[3 + i * 100] * 256 + bufaddr[2 + i * 100]) / 100;
                this->parsed_packed.timestamp = (bufaddr[1203] << 24 | bufaddr[1202] << 16 | bufaddr[1201] << 8 | bufaddr[1200]) / 1000000;
            }
            catch (const std::exception&){
                printf("Bad UDP packet received!");
                exit(0);
            }

            if (new_azimuth < this->prv_azimuth) {
                this->azimuth_gap = float((new_azimuth + 360) - this->prv_azimuth);

            }

            this->prv_azimuth = new_azimuth;
            
            for (int j = 0; j < (MAX_CHANNEL * 3); j+=3) {
                this->parsed_packed.gelenpaket[i].channel[z].distance = ((bufaddr[i * 100 + j + 5] * 256 + bufaddr[i * 100 + j + 4]) * 2) / (float)10000;
                this->parsed_packed.gelenpaket[i].channel[z].intensity = bufaddr[i * 100 + j + 6];
                real_laser = laser_id % 16;

                if (laser_id < 16)
                    new_azimuth = new_azimuth + float(this->azimuth_gap * 2.304 * real_laser / 55.296) / 10;
                else
                    new_azimuth = new_azimuth + float(this->azimuth_gap * 2.304 * real_laser / 55.296) / (2 * 55.296) / 10;

                if (new_azimuth >= (float)360)
                    new_azimuth = new_azimuth - (float)360;

                this->parsed_packed.gelenpaket[i].channel[this->laser_id].azimuth = new_azimuth;
                this->parsed_packed.gelenpaket[i].channel[this->laser_id].laser_id = real_laser;

                z += 1;
                laser_id += 1;
            }
        }
    };

    DataFrame getDataBlock(void) {
        return this->parsed_packed;
    };

    int getFrameIndex(void) {
        return this->frame;
    };

protected:
    DataFrame   parsed_packed = {0};
    float       prv_azimuth = 0.0;
    float       azimuth_gap = 0.0;
    int         frame = 0;
    float       new_azimuth = 0.0;
    
};

/*! \brief GPGGA Parser
 * Onyle used for NME0183 GGA parsing process
 */
class Parsegga{

public:
    void setGGA(string ggatext) {
    //TODO
    }

    float getLongitude(void) {
        //TODO
    }

    float getLatitude(void) {
        //TODO
    }
};

/*! \brief PKHM Parser
 * Onyl used for NME0183 PKHM parsing process
 */
class Parsepkhm {

public:
    void setPKHM(string pkhmtext) {
        string delimiter = ",";
        
        vector<string> v = split(pkhmtext, delimiter);

        try
        {
            this->pitch = ::atof(v[6].c_str());
            this->roll = ::atof(v[5].c_str());
            this->yaw = ::atof(v[7].c_str());

            this->timeslap = ::atof(v[1].c_str());

        }
        catch (const std::exception&)
        {
            printf("\nPKHM parsing error!");
        }
        //std::cout << "\nRoll: " << v[5] << " Pitch" << v[6] << " Yaw" << v[7];
    }

    float getPitch(void) {
        return this->pitch;
    }

    float getRoll(void) {
        return this->roll;
    }

    float getYaw(void) {
        return this->yaw;
    }

    float getTimeslap(void) {
        return this->timeslap;
    }

protected:
    float pitch;
    float roll;
    float yaw;
    float timeslap;
};

Parsegga* mygga;
Parseudp* mypoints;
Parsepkhm* mypkhm;
Util* myutil;



/*! \brief Develop point cloud
 * Fills point cloud buffer with point distance and intensity
 */
void fill_buffer(void) {
    
    for (int i = 0; i < MAX_NUMS; i++) {
        for (int j = 0; j < (MAX_CHANNEL); j++) {
            if (previous_azimuth > mypoints->getDataBlock().gelenpaket[i].channel[j].azimuth) {
                point_frame[packidx].laser_id = 99;

                myinfo.point_amount = packidx;
                packidx = 0;
            }

            previous_azimuth = mypoints->getDataBlock().gelenpaket[i].channel[j].azimuth;

            point_frame[packidx].distance = mypoints->getDataBlock().gelenpaket[i].channel[j].distance;
            point_frame[packidx].azimuth = mypoints->getDataBlock().gelenpaket[i].channel[j].azimuth;
            point_frame[packidx].intensity = mypoints->getDataBlock().gelenpaket[i].channel[j].intensity;
            point_frame[packidx].laser_id = mypoints->getDataBlock().gelenpaket[i].channel[j].laser_id;

            packidx++;
        }
    }
    
    if (packidx >= (35999)) //fuse
        packidx = 0;
}

/*! \brief OpenGL Draw point cloud
 * Draws point and refresh
 */
void DrawPointCloud()
{
    int idx = 0;
    float x, y, z;

    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_POINTS);
    while (point_frame[idx].laser_id != 99) {
        tie (x,y,z) = myutil->getXYZ(point_frame[idx].distance, point_frame[idx].azimuth, point_frame[idx].laser_id);
        glVertex3f(x, y, z);
        idx++;
    }

    //glColor3f(rgb.r, rgb.g, rgb.b);
    //glRasterPos2f(x, y);

    //glutBitmapCharacter(GLUT_BITMAP_TIMES_ROMAN_24, '3');


    glEnd();

    glFlush();
}

void mouseCB(int button, int state, int x, int y)
{
    widget.mouseX = x;
    widget.mouseY = y;
    times = 1;

    if (button == GLUT_LEFT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            widget.mouseLeftDown = true;
        }
        else if (state == GLUT_UP)
            widget.mouseLeftDown = false;
    }

    else if (button == GLUT_RIGHT_BUTTON)
    {
        if (state == GLUT_DOWN)
        {
            widget.mouseRightDown = true;
        }
        else if (state == GLUT_UP)
            widget.mouseRightDown = false;
    }
    else {
        widget.mouseLeftDown = false;
        widget.mouseRightDown = false;
    }

}

void mouseMotionCB(int x, int y)
{
    widget.cameraAngleX = widget.cameraAngleY = 0;
    widget.cameraDistanceX = widget.cameraDistanceY = 0;

    if (widget.mouseLeftDown)
    {
        widget.cameraAngleY += (x - widget.mouseX) * 0.1f;
        widget.cameraAngleX += (y - widget.mouseY) * 0.1f;
        widget.mouseX = x;
        widget.mouseY = y;
    }
    if (widget.mouseRightDown)
    {
        widget.cameraDistanceX = (x - widget.mouseX) * 0.002f;
        widget.cameraDistanceY = -(y - widget.mouseY) * 0.002f;
        widget.mouseY = y;
        widget.mouseX = x;
    }

    glutPostRedisplay();
}
/*! \brief OpenGL update Display
 * Draw point cloud
 */
void display()
{
    glColor3f(0.0, 1.0, 0.0);
    glPointSize(1);

    glScalef(times, times, times);//Zoom
    glTranslatef(widget.cameraDistanceX, widget.cameraDistanceY, 0);
    glRotatef(widget.cameraAngleX, 1, 0, 0);
    glRotatef(widget.cameraAngleY, 0, 1, 0);

    DrawPointCloud();

    glutSwapBuffers();
}

/*! \brief Draw text
 * Draw text specific coordinate
 */
void drawText(const char* text, int length, int x, int y) {

    glMatrixMode(GL_PROJECTION);
    double* matrix = new double[16];
    glGetDoublev(GL_PROJECTION_MATRIX, matrix);
    glLoadIdentity();
    glOrtho(0, 800, 0, 600, -5, 5);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    glPushMatrix();
    glLoadIdentity();
    glRasterPos2i(x, y);
    for (int i = 0; i < length; i++) {
        glutBitmapCharacter(GLUT_BITMAP_9_BY_15, (int)text[i]);
    }
    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixd(matrix);
    glMatrixMode(GL_MODELVIEW);
}

/*! \brief OpenGL update Child Display
 * Draw Child Display
 */
void display_sub()
{
    glColor3f(0.0, 1.0, 0.0);
    glPointSize(1);

    string text;
    text = "TINY IMU";

    std::string pitchtext = "Pitch: ";
    pitchtext += std::to_string(myimu.pitch);
    std::string rolltext = "Roll: ";
    rolltext += std::to_string(myimu.roll);
    std::string yawtext = "Yaw: ";
    yawtext += std::to_string(myimu.yaw);
    std::string timestamptext = "Timestamp: ";
    timestamptext += std::to_string(myimu.timestamp);


    string textld;
    textld = "LIDAR";

    std::string pointtext = "Points: ";
    pointtext += std::to_string(myinfo.point_amount);


    glClear(GL_COLOR_BUFFER_BIT);

    drawText(text.data(), text.size(), 0, 500);
    drawText(pitchtext.data(), pitchtext.size(), 0, 430);
    drawText(rolltext.data(), rolltext.size(), 0, 380);
    drawText(yawtext.data(), yawtext.size(), 0, 330);
    drawText(timestamptext.data(), timestamptext.size(), 0, 280);

    drawText(textld.data(), textld.size(), 0, 180);
    drawText(pointtext.data(), pointtext.size(), 0, 110);


    /*glPointSize(2);
    glBegin(GL_LINES);
    glColor3f(1, 1, 0);
    glVertex3f(-0.5, -0.04, -0.2);
    glVertex3f(0.5, -0.04, -0.2);
    glEnd();
    glFlush();*/


    /*glPointSize(2);
    glBegin(GL_LINES);
    glColor3f(1, 0, 0);
    glVertex3f(-0.5, -0.02, -0.2);
    glVertex3f(0.5, -0.02, -0.2);

    glColor3f(0, 1, 0);
    glVertex3f(0, -0.5, -0.2);
    glVertex3f(0, 0.5, -0.2);

    glColor3f(0, 0, 1);
    glVertex3f(0, -0.02, -0.5);
    glVertex3f(0, -0.02, 0);*/

    //glRasterPos3f(1, 0, 0);
    //char string[] = "Hello!";
    //for (int i = 0; string[i] != '\0'; i++)
    //    glutBitmapCharacter(GLUT_BITMAP_9_BY_15, string[i]);

    //glEnd();
    //glFlush();

    glutSwapBuffers();
}
/*! \brief OpenGL refresh display
 * Reshape canvas according to new dimension.
 */
void reshape(GLsizei width, GLsizei height) {

    if (height == 0) height = 1; //prevent divide by zero
    GLfloat aspect = (GLfloat)width / (GLfloat)height;

    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);  // To operate on the Projection matrix
    glLoadIdentity();
    // Enable perspective projection with fovy, aspect, zNear and zFar
    //gluPerspective(20.0f, aspect, 4.0f, 5.0f);
    /*gluLookAt(0, -4, 1,
                0, 0, 0, 
                0, 1, 0);*/
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void keyboard(unsigned char key, int x, int y)
{
    if (key == 'w')
    {
        times = 0.008f + 1;
        glutPostRedisplay();
    }
    else if (key == 's')
    {
        times = -0.008f + 1;
        glutPostRedisplay();
    }
   
}

/*! \brief OpenGL update Timer
 * Refresh canvas object.
 */
void timer(int value) {
    static uint8_t tout = 0;

    glutSetWindow(sub_w);
    glutPostRedisplay();                    // Post re-paint request to activate display()
    glutSetWindow(main_w);
    glutPostRedisplay();                    // Post re-paint request to activate display()
    glutTimerFunc(refreshMills, timer, 0);  // next timer call milliseconds later

    if (++tout >= 10) {
        tout = 0;

        times = 1;
        widget.cameraAngleX = widget.cameraAngleY = 0;
        widget.cameraDistanceX = widget.cameraDistanceY = 0;
    }
}

/*! \brief OpenGL Task. 
 * Called from 2. thread.
 */
void taskOpenGL(void *a) {

    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
    glutInitWindowSize(1024, 768);
    glutInitWindowPosition(10, 10);
    
    main_w = glutCreateWindow("Kindhelm LIDAR-IMU Visualisation");
    glutDisplayFunc(&display);

    sub_w = glutCreateSubWindow(main_w, 10, 10, 150, 200);
    glutSetWindow(sub_w);
    glutDisplayFunc(&display_sub);
    glutReshapeFunc(&reshape);

    glutSetWindow(main_w);
    glutKeyboardFunc(keyboard);
    glutMouseFunc(mouseCB);
    glutMotionFunc(mouseMotionCB);
    glutTimerFunc(0, timer, 0);

    glutMainLoop();

};

/*! \brief Serial comm Task.
 * Called from 1. thread.
 */
void taskUart(void* a) {

#ifdef   APP_WINDOWS
    serialInit("\\\\.\\COM7"); //TODO must be parametric. :/

    while (1) {
        serialLoop();

        mypkhm->setPKHM(SerialBuffer);
        
        myimu.pitch = mypkhm->getPitch();
        myimu.roll = mypkhm->getRoll();
        myimu.yaw = mypkhm->getYaw();
        myimu.timestamp = mypkhm->getTimeslap();
    }
#else

#endif



};


/*! \brief Listens UDP and serial
 * Parse and collect point 3d vector
 * Show space with OPENGL
 */
int main(int argc, char** argv) {

    int iCPU = omp_get_num_procs();
    omp_set_num_threads(iCPU);

    std::cout << "Kindhelm! " << iCPU << " cores detected and threaded!";

    mygga = new Parsegga();
    mypoints = new Parseudp();
    mypkhm = new Parsepkhm();
    myutil = new Util();

    glutInit(&argc, argv);
    _beginthread(taskOpenGL, 0, (void*)0);
    //_beginthread(taskUart, 0, (void*)0);  //comment out if Tiny is connected over Serial.


    //Test GGA
    mygga->setGGA("$GPGGA,134658.00,2911.15,N,4054.569988,E,2,09,1.0,1048.47,M,-16.27,M,08,AAAA*60\r\n");

    //Test PKHM
    mypkhm->setPKHM("$PKHM, 000916.48, 01, 01, 1970, 3.051690, 356.875946, 182.506226 * 28");

    sockaddr_in server, client;

    WSADATA wsa;
    printf("Initialising Winsock...");
    if (WSAStartup(MAKEWORD(2, 2), &wsa) != 0){
        printf("Failed. Error Code: %d", WSAGetLastError());
        exit(0);
    }
    printf("Initialised.\n");

    SOCKET server_socket;
    if ((server_socket = socket(AF_INET, SOCK_DGRAM, 0)) == INVALID_SOCKET){
        printf("Could not create socket: %d", WSAGetLastError());
    }
    printf("Socket created.\n");


    server.sin_family = AF_INET;
    server.sin_addr.s_addr = INADDR_ANY;
    server.sin_port = htons(PORT);


    if (bind(server_socket, (sockaddr*)&server, sizeof(server)) == SOCKET_ERROR)
    {
        printf("Bind failed with error code: %d", WSAGetLastError());
        exit(EXIT_FAILURE);
    }
    puts("Bind done.");


    while (true)
    {
           
        fflush(stdout);
        char message[BUFLEN] = {};

        int message_len;
        int slen = sizeof(sockaddr_in);

        if (message_len = recvfrom(server_socket, message, BUFLEN, 0, (sockaddr*)&client, &slen) == SOCKET_ERROR){
            printf("recvfrom() failed with error code: %d", WSAGetLastError());
            exit(0);
        }
        else {
            mypoints->setUDP((uint8_t*)message);

            fill_buffer();
        }

        
    }
    return 0;
}