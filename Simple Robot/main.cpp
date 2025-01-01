/*This source code copyrighted by Lazy Foo' Productions 2004-2024
 and may not be redistributed without written permission.*/

//Using SDL and standard IO
#include <SDL2/SDL.h>
#include <SDL2/SDL_rect.h>
#include <SDL2/SDL_surface.h>
#include <SDL2/SDL_opengl.h>

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <stdio.h>

//#include "joint.h"

#include "./imgui/imgui.h"
#include "./imgui/backends/imgui_impl_sdl2.h"
#include "./imgui/backends/imgui_impl_opengl2.h"

//#include "robot.h"
#include "myConstants.h"

//class RevolutJoint: public Joint {
//public:
//    RevolutJoint(): Joint()
//    {}
//    
//};
//
//class EndEffector: public Compoment {
//public:
//    EndEffector(std::string fmt, float w, float h):
//    Compoment(fmt, w, h) {}
//};
//

//class SimpleRobot final {
//public:
//    SimpleRobot()
//    {}
//    
//    void connect(Joint* j, Body* b) {
//        if(!jointVector.empty() || !bodyVector.empty()){
//            j->connect(b);
//            //b->setPosition(j->getX(), j->getY() + b->getHeight() / 2);
//            b->isChild = true;
//            j->children.push_back(b);
//        }
//    }
//    void connect(Body* b, Joint* j) {
//        std::cout << "Connect joint to body" << std::endl;
//        if(!jointVector.empty() || !bodyVector.empty()){
//            j->connect(b);
//            //j->setPosition(b->getShape().topLeft.x, b->getY() + b->getHeight() / 2);
//            j->isChild = true;
//            b->children.push_back(j);
//        }
//    }
//    
//    Body* addBody() {
//        Body* b = new Body(std::string("Body") + std::to_string(bodyIdx),body_width, body_height);
//        bodyVector.push_back(b);
//        bodyIdx++;
//        return b;
//    }
//    
//    EndEffector* addEndEffector() {
//        EndEffector* e = new EndEffector(std::string("Body") + std::to_string(bodyIdx),body_width, body_height);
//        return e;
//    }
//    
//    Joint* addJoint() {
//        Joint* j = new Joint(std::string("Joint") + std::to_string(jointIdx), joint_radius);
//        jointVector.push_back(j);
//        jointIdx++;
//        return j;
//    }
//    
//    void update() {
//        for (Body* b : bodyVector) {
//            b->update();
//        }
//        
//        for (Joint* j : jointVector) {
//            j->update();
//        }
//    }
//    
//    std::vector<Body*> getBodies() {
//        return bodyVector;
//    }
//    std::vector<Joint*> getJoints() {
//        return jointVector;
//    }
//    
//    void draw(ImDrawList* dl) {
//        for (Body* b : bodyVector) {
//            ImVec2 top_left = ImVec2(b->getShape().topLeft.x, b->getShape().topLeft.y);
//            ImVec2 top_right = ImVec2(b->getShape().topRight.x,b->getShape().topRight.y);
//            ImVec2 bottom_right = ImVec2(b->getShape().bottomRight.x, b->getShape().bottomRight.y);
//            ImVec2 bottom_left =  ImVec2(b->getShape().bottomLeft.x,b->getShape().bottomLeft.y);
//            
//            //            ImVec2 top_left = ImVec2(windowPos.x + b->getWidth() + b->getX(), windowPos.y + b->getHeight() + b->getY());
//            //            ImVec2 top_right = ImVec2(windowPos.x + b->getX(), windowPos.y + b->getHeight() + b->getY());
//            //            ImVec2 bottom_left = ImVec2(windowPos.x + b->getWidth() + b->getX(),windowPos.y + b->getY());
//            //            ImVec2 bottom_right = ImVec2(windowPos.x + b->getX(), windowPos.y + b->getY());
//            
//            dl->AddQuadFilled(top_left, top_right, bottom_right, bottom_left, b->getColor());
//        }
//        for (Joint* j : jointVector) {
//            ImVec2 center = ImVec2(j->getX(), j->getY());
//            
//            dl->AddCircleFilled(center, j->getRadius(), j->getColor());
//            dl->AddLine(center, ImVec2(center.x + j->getRadius(), center.y), IM_COL32(0, 255, 0, 255), 5);
//        }
//    }
//    
//    
//    void rotate(Body* b, float angle, Eigen::Vector2f pivot) {
//        //b->setAngle(angle);
//        b->setPivot(pivot);
//    }
//    
//private:
//    int bodyIdx = 1;
//    int jointIdx = 1;
//    int body_width = 20;
//    int body_height = 100;
//    int joint_radius = 20;
//    
//    std::vector<Body*> bodyVector;
//    std::vector<Joint*> jointVector;
//    
//};



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
    
    //Robot robot = Robot("Wall-e");
    int mouseX = 0;
    int mouseY = 0;
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
            
            // Mouse click events
            if (event.type == SDL_MOUSEBUTTONDOWN)
            {
                mouseX = event.button.x;
                mouseY = event.button.y;
                
            }
            
            // Optional: Mouse button release events
            if (event.type == SDL_MOUSEBUTTONUP)
            {
                switch (event.button.button)
                {
                    case SDL_BUTTON_LEFT:
                        // Left mouse button released
                        break;
                        
                    case SDL_BUTTON_RIGHT:
                        // Right mouse button released
                        break;
                }
            }
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
            
            ImGui::SetWindowPos(ImVec2(SCREEN_WIDTH / 2, 0));
            ImGui::SetWindowSize(ImVec2(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2));
            
//            if(ImGui::Button("Add link")) {
//                std::shared_ptr<Link> link(new Link("Link 1"));
//                robot.addLink(link);
//            }
//            ImGui::SameLine();
//            if(ImGui::Button("Add joint")) {
//                std::shared_ptr<PrismaticJoint> joint(new PrismaticJoint("Joint 1"));
//                //robot.addJoint(joint);
//            }
//            ImGui::Spacing();
//            ImGui::Combo("Bodies", &current_body, "body1\0body2\0body3\0\0");
//            ImGui::Combo("Joints", &current_joint, "joint1\0joint2\0\0");
//            ImGui::Spacing();
//            if(ImGui::Button("Connect body to joint")) {
//                robot.connect(robot.getJoints().operator[](current_joint), robot.getBodies().operator[](current_body));
//                
//            }
//            ImGui::SameLine();
//            if(ImGui::Button("Connect joint to body")) {
//                robot.connect(robot.getBodies().operator[](current_body), robot.getJoints().operator[](current_joint));
//            }
//            if(robot.getBodies().size() != 0 && robot.getJoints().size() != 0){
//                robot.rotate(robot.getBodies().operator[](current_body), robot.getJoints().operator[](current_joint)->getAngle(), Eigen::Vector2f(robot.getJoints().operator[](current_joint)->getX(), robot.getJoints().operator[](current_joint)->getY()));
//            }
//            
//            robot.update();
            
            
            ImGui::End();
        }
        {
            ImGui::Begin("Drawing", NULL, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);
            ImGui::SetWindowPos(ImVec2(0, 0));
            ImGui::SetWindowSize(ImVec2(SCREEN_WIDTH / 2, SCREEN_HEIGHT));
            ImDrawList* dl = ImGui::GetWindowDrawList();
            dl->AddLine(ImVec2(mouseX, mouseY), ImVec2(mouseX, mouseY - 50), IM_COL32_WHITE, 5);
            dl->AddTriangleFilled(ImVec2(mouseX - 10, mouseY - 50), ImVec2(mouseX, mouseY - 70), ImVec2(mouseX + 10, mouseY - 50), IM_COL32_WHITE);
            //robot.draw(ImGui::GetWindowDrawList());
            
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

