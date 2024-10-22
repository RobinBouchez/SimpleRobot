/*This source code copyrighted by Lazy Foo' Productions 2004-2024
 and may not be redistributed without written permission.*/

//Using SDL and standard IO
#include <SDL2/SDL.h>
#include <SDL2/SDL_rect.h>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_opengl.h>

#include <iostream>
#include <string>
#include <vector>
#include <cmath>

#include "./imgui/imgui.h"
#include "./imgui/backends/imgui_impl_sdl2.h"
#include "./imgui/backends/imgui_impl_opengl2.h"

#include "Eigen/Dense"
#include "Eigen/Geometry"

//Screen dimension constants
const int SCREEN_WIDTH = 1280;
const int SCREEN_HEIGHT = 720;

class Compoment;
class Joint;
class Body;

struct Shape{
    ImVec2 topLeft, topRight, bottomLeft, bottomRight;
};


class Compoment {
public:
    Compoment(std::string fmt, float w, float h):
    width(w), height(h), name(fmt),  x(0), y(0)
    {
        color = IM_COL32(rand() % 255, rand() % 255, rand() % 255, 255);
    }
    
    
    int getWidth() const { return width; }
    int getHeight() const { return height; }
    float getX() const { return x; }
    float getY() const { return y; }
    float getAngle() const { return angle; }
    Shape getShape() {
        return shape;
    }
    
    bool fixed = false;
    bool isChild = false;
    
    
    void setPosition(float newX, float newY) {
        x = newX;
        y = newY;
    }
    
    void setAngle(float newAngle) { angle = newAngle; }
    void setShape(ImVec2 newtopLeft, ImVec2 newtopRight, ImVec2 newbottomLeft, ImVec2 newbottomRight) {
        shape.topLeft = newtopLeft;
        shape.topRight = newtopRight;
        shape.bottomLeft = newbottomLeft;
        shape.bottomRight = newbottomRight;
    }
    
    std::string getName() const { return name; }
    
    ImU32 getColor() const { return color; }
    float angle;
    
    Eigen::Vector2f pivot;
    
    void setPivot(Eigen::Vector2f newPivot) { pivot = newPivot; }
    
    std::vector<Compoment*> children;
    
protected:
    Shape shape;
    ImU32 color;
    std::string name;
    float width;
    float height;
    float x;
    float y;
};

class Body: public Compoment {
public:
    Body(std::string fmt, float w, float h):
    Compoment(fmt, w, h) {}
    
    void update() {
        ImGui::Begin(name.c_str());
        
        ImGui::BeginDisabled(fixed || isChild);
        ImGui::SliderFloat("x", &x, 0.0f, SCREEN_WIDTH / 2);
        ImGui::SliderFloat("y", &y, 0.0f, SCREEN_HEIGHT);
        ImGui::EndDisabled();
        
        ImGui::SliderFloat("Width", &width, 0.0f, 200.0f);
        ImGui::SliderFloat("Height", &height, 0.0f, 200.0f);
        ImGui::SliderFloat("angle", &angle, 0.0f, EIGEN_PI * 2);
        ImGui::Spacing();
        
        ImGui::Checkbox("Is component fixed ", &fixed);
        
        ImGui::End();
        
        Eigen::Matrix<float, 2, 4> rectangle;
        rectangle << x - width / 2,  x + width / 2, x + width / 2, x - width / 2,
        y - height / 2, y - height / 2, y + height / 2, y + height / 2;
        
        // Create the 2D rotation matrix
        Eigen::Matrix2f rotation;
        rotation << std::cos(angle), -std::sin(angle),
        std::sin(angle),  std::cos(angle);
        
        Eigen::Vector2f p(x, y);
        //p += pivot;
        
        Eigen::Matrix<float, 2, 4> translatedRectangle = rectangle.colwise() - pivot;
        
        Eigen::Matrix<float, 2, 4> rotatedRectangle = rotation * translatedRectangle;
        
        Eigen::Matrix<float, 2, 4> finalRectangle = rotatedRectangle.colwise() + pivot;
        
        
        this->setShape(ImVec2(finalRectangle(0, 0), finalRectangle(1, 0)),
                       ImVec2(finalRectangle(0, 1), finalRectangle(1, 1)),
                       ImVec2(finalRectangle(0, 2), finalRectangle(1, 2)),
                       ImVec2(finalRectangle(0, 3), finalRectangle(1, 3)));
        
        for (Compoment* child: children){
            child->setPosition(this->shape.bottomRight.x + abs(this->shape.bottomLeft.x - this->shape.bottomRight.x) / 2,
                               this->shape.bottomLeft.y + abs(this->shape.bottomLeft.y - this->shape.bottomRight.y) / 2);
            child->setPivot(Eigen::Vector2f(x, y));
        }
    }
};


class Joint: public Compoment {
public:
    Joint(std::string fmt, float w, float h): Compoment(fmt, w, h) {}
    
    void update() {
        ImGui::Begin(name.c_str());
        
        ImGui::BeginDisabled(fixed || isChild);
        ImGui::SliderFloat("x", &x, 0.0f, SCREEN_WIDTH / 2);
        ImGui::SliderFloat("y", &y, 0.0f, SCREEN_HEIGHT);
        ImGui::EndDisabled();
        
        ImGui::SliderFloat("Radius", &radius, 10.0f, 100.0f);
        ImGui::Spacing();
        
        ImGui::End();
        
        width = height = radius;
        
        this->setShape(ImVec2(x - radius, y + radius), ImVec2(x + radius, y + radius), ImVec2(x - radius, y - radius), ImVec2(x + radius, y - radius));
        
        for (Compoment* child: children){
            child->setPosition(this->shape.bottomRight.x + abs(this->shape.bottomLeft.x - this->shape.bottomRight.x) / 2,
                               this->shape.bottomLeft.y + abs(this->shape.bottomLeft.y - this->shape.bottomRight.y) / 2);
            child->setPivot(Eigen::Vector2f(x, y));
        }
        
    }
    
    float radius = 10.0f;
    
};

class EndEffector: public Compoment {
public:
    EndEffector(std::string fmt, float w, float h):
    Compoment(fmt, w, h) {}
};


class SimpleRobot final {
public:
    SimpleRobot(float x_pos, float y_pos):
    x(x_pos), y(y_pos)
    {}
    
    void connect(Joint* j, Body* b) {
        if(!jointVector.empty() || !bodyVector.empty()){
            b->setPosition(j->getX(), j->getY() + b->getHeight() / 2);
            b->isChild = true;
            j->children.push_back(b);
        }
    }
    void connect(Body* b, Joint* j) {
        if(!jointVector.empty() || !bodyVector.empty()){
            j->setPosition(b->getX(), b->getY() + b->getHeight() / 2);
            j->isChild = true;
            b->children.push_back(j);
        }
    }
    
    Body* addBody() {
        Body* b = new Body(std::string("Body") + std::to_string(bodyIdx),body_width, body_height);
        bodyVector.push_back(b);
        bodyIdx++;
        return b;
    }
    
    EndEffector* addEndEffector() {
        EndEffector* e = new EndEffector(std::string("Body") + std::to_string(bodyIdx),body_width, body_height);
        return e;
    }
    
    Joint* addJoint() {
        Joint* j = new Joint(std::string("Joint") + std::to_string(jointIdx),joint_width, joint_height);
        jointVector.push_back(j);
        jointIdx++;
        return j;
    }
    
    void update() {
        for (Body* b : bodyVector) {
            b->update();
        }
        
        for (Joint* j : jointVector) {
            j->update();
        }
    }
    
    std::vector<Body*> getBodies() {
        return bodyVector;
    }
    std::vector<Joint*> getJoints() {
        return jointVector;
    }
    
    void draw(ImDrawList* dl, ImVec2 windowPos) {
        for (Body* b : bodyVector) {
            ImVec2 top_left = ImVec2(b->getShape().topLeft.x + windowPos.x,b->getShape().topLeft.y + windowPos.y);
            ImVec2 top_right = ImVec2(b->getShape().topRight.x + windowPos.x,b->getShape().topRight.y + windowPos.y);
            ImVec2 bottom_right = ImVec2(b->getShape().bottomRight.x + windowPos.x,b->getShape().bottomRight.y + windowPos.y);
            ImVec2 bottom_left =  ImVec2(b->getShape().bottomLeft.x + windowPos.x,b->getShape().bottomLeft.y + windowPos.y);
            
            //            ImVec2 top_left = ImVec2(windowPos.x + b->getWidth() + b->getX(), windowPos.y + b->getHeight() + b->getY());
            //            ImVec2 top_right = ImVec2(windowPos.x + b->getX(), windowPos.y + b->getHeight() + b->getY());
            //            ImVec2 bottom_left = ImVec2(windowPos.x + b->getWidth() + b->getX(),windowPos.y + b->getY());
            //            ImVec2 bottom_right = ImVec2(windowPos.x + b->getX(), windowPos.y + b->getY());
            
            dl->AddQuadFilled(top_left, top_right, bottom_left, bottom_right, b->getColor());
        }
        for (Joint* j : jointVector) {
            ImVec2 center = ImVec2(windowPos.x + j->getX(), windowPos.y + j->getY());
            
            dl->AddCircleFilled(center, j->radius, j->getColor());
        }
    }
    
    
    void rotate(Body* b, float angle, Eigen::Vector2f pivot) {
        //b->setAngle(angle);
        b->setPivot(pivot);
    }
    
private:
    
    float x;
    float y;
    int bodyIdx = 1;
    int jointIdx = 1;
    int body_width = 20;
    int body_height = 100;
    int joint_width = 20;
    int joint_height = 100;
    
    std::vector<Body*> bodyVector;
    std::vector<Joint*> jointVector;
    
};

int main( int argc, char* args[] )
{
    // Setup SDL
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER | SDL_INIT_GAMECONTROLLER) != 0)
    {
        printf("Error: %s\n", SDL_GetError());
        return -1;
    }
    
    // From 2.0.18: Enable native IME.
#ifdef SDL_HINT_IME_SHOW_UI
    SDL_SetHint(SDL_HINT_IME_SHOW_UI, "1");
#endif
    
    // Setup window
    SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
    SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
    SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, 8);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
    SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 2);
    SDL_WindowFlags window_flags = (SDL_WindowFlags)(SDL_WINDOW_OPENGL | SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    SDL_Window* window = SDL_CreateWindow("Simple Robot", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, SCREEN_WIDTH, SCREEN_HEIGHT, window_flags);
    if (window == nullptr)
    {
        printf("Error: SDL_CreateWindow(): %s\n", SDL_GetError());
        return -1;
    }
    
    SDL_GLContext gl_context = SDL_GL_CreateContext(window);
    SDL_GL_MakeCurrent(window, gl_context);
    SDL_GL_SetSwapInterval(1); // Enable vsync
    
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls
    
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();
    
    // Setup Platform/Renderer backends
    ImGui_ImplSDL2_InitForOpenGL(window, gl_context);
    ImGui_ImplOpenGL2_Init();
    
    ImVec4 clear_color = ImVec4(1, 1, 1, 1.00f);
    
    SimpleRobot robot = SimpleRobot(SCREEN_WIDTH / 2, 0);
    
    // Main loop
    bool done = false;
    while (!done)
    {
        SDL_Event event;
        while (SDL_PollEvent(&event))
        {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT)
                done = true;
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE && event.window.windowID == SDL_GetWindowID(window))
                done = true;
        }
        if (SDL_GetWindowFlags(window) & SDL_WINDOW_MINIMIZED)
        {
            SDL_Delay(10);
            continue;
        }
        
        // Start the Dear ImGui frame
        ImGui_ImplOpenGL2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();
        
        
        {
            ImGui::Begin("Controls");
            
            
            static int current_body = 0;
            static int current_joint = 0;
            
            ImGui::SetWindowPos(ImVec2(0, 0));
            ImGui::SetWindowSize(ImVec2(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2));
            
            if(ImGui::Button("Add body")) {
                robot.addBody();
            }
            ImGui::SameLine();
            if(ImGui::Button("Add Joint")) {
                robot.addJoint();
            }
            ImGui::Spacing();
            ImGui::Combo("Bodies", &current_body, "body1\0body2\0body3\0\0");
            ImGui::Combo("Joints", &current_joint, "joint1\0joint2\0\0");
            ImGui::Spacing();
            if(ImGui::Button("Connect body to joint")) {
                robot.connect(robot.getJoints().operator[](current_joint), robot.getBodies().operator[](current_body));
                
            }
            ImGui::SameLine();
            if(ImGui::Button("Connect joint to body")) {
                robot.connect(robot.getBodies().operator[](current_body), robot.getJoints().operator[](current_joint));
            }
            if(robot.getBodies().size() != 0 && robot.getJoints().size() != 0){
                robot.rotate(robot.getBodies().operator[](current_body), robot.getJoints().operator[](current_joint)->getAngle(), Eigen::Vector2f(robot.getJoints().operator[](current_joint)->getX(), robot.getJoints().operator[](current_joint)->getY()));
            }
            
            robot.update();
            
            
            ImGui::End();
        }
        {
            ImGui::Begin("Drawing", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
            ImGui::SetWindowPos(ImVec2(SCREEN_WIDTH / 2, 0));
            ImGui::SetWindowSize(ImVec2(SCREEN_WIDTH / 2, SCREEN_HEIGHT));
            
            robot.draw(ImGui::GetWindowDrawList(), ImGui::GetWindowPos());
            
            ImGui::End();
        }
        
        
        // Rendering
        ImGui::Render();
        glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        //glUseProgram(0); // You may want this if using this code in an OpenGL 3+ context where shaders may be bound
        ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());
        SDL_GL_SwapWindow(window);
    }
    
    // Cleanup
    ImGui_ImplOpenGL2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    
    SDL_GL_DeleteContext(gl_context);
    SDL_DestroyWindow(window);
    SDL_Quit();
    
    return 0;
}
