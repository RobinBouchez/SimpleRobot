#include "body.h"


Body::Body(std::string fmt, float w, float h):
Compoment(fmt, w, h) {}

void Body::update() {
    ImGui::Begin(name.c_str());
    
    ImGui::BeginDisabled(fixed);
    ImGui::SliderFloat("x", &x, 0.0f, SCREEN_WIDTH / 2);
    ImGui::SliderFloat("y", &y, 0.0f, SCREEN_HEIGHT);
    ImGui::EndDisabled();
    
    ImGui::SliderFloat("Width", &width, 0.0f, 200.0f);
    ImGui::SliderFloat("Height", &height, 0.0f, 200.0f);
    ImGui::SliderFloat("angle", &angle, 0.0f, EIGEN_PI * 2);
    ImGui::Spacing();
    
    ImGui::Checkbox("Is component fixed ", &fixed);
    
    Eigen::Matrix<float, 2, 4> rectangle;
    rectangle << x - width / 2, x + width / 2, x + width / 2, x - width / 2,
    y - height / 2, y - height / 2, y + height / 2, y + height / 2;
    
//    
//    Eigen::Matrix2f rotation;
//    rotation << std::cos(angle), -std::sin(angle),
//    std::sin(angle),  std::cos(angle);
//    
//    Eigen::Vector2f pos(x, y);
//    setPivot(Eigen::Vector2f(shape.topLeft.x, shape.topLeft.y));
//    
//    Eigen::Matrix<float, 2, 4> translatedRectangle = rectangle.colwise() - pivot;
//    
//    Eigen::Matrix<float, 2, 4> rotatedRectangle = rotation * translatedRectangle;
//    
//    pos += pivot;
//    Eigen::Matrix<float, 2, 4> finalRectangle = rotatedRectangle.colwise() + pos;
//    
//    ImGui::Text("Pivot x %f", pivot.x());
//    ImGui::Text("Pivot x %f", pivot.y());
    
    
    this->setShape(ImVec2(rectangle(0, 0), rectangle(1, 0)),
                   ImVec2(rectangle(0, 1), rectangle(1, 1)),
                   ImVec2(rectangle(0, 2), rectangle(1, 2)),
                   ImVec2(rectangle(0, 3), rectangle(1, 3)));
    
//    if (ImGui::BeginTable("matrix", 4))
//    {
//        for (int row = 0; row < 2; row++)
//        {
//            ImGui::TableNextRow();
//            for (int column = 0; column < 4; column++)
//            {
//                ImGui::TableSetColumnIndex(column);
//                ImGui::Text("%f", finalRectangle(row, column));
//            }
//        }
//        ImGui::EndTable();
//    }
    
//    for (Compoment* child: children){
//        child->setPosition(shape.topLeft.x + (abs(shape.topLeft.x - shape.topRight.x) / 2),
//                           shape.topLeft.y + (abs(shape.topLeft.y - shape.topRight.y) / 2));
//        setPivot(Eigen::Vector2f(x, y));
//    }
    ImGui::End();
}
