#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include "raylib.h"
// #include <cv_bridge/cv_bridge.h>

#define CLAY_IMPLEMENTATION
#include "igvc_display/clay.h"
#include "igvc_display/clayrib.h"

// helpful constants and helpers
#define RAYLIB_VECTOR2_TO_CLAY_VECTOR2(vector) (Clay_Vector2) { .x = vector.x, .y = vector.y }
#define COLOR_ORANGE (Clay_Color) {225, 138, 50, 255}
#define COLOR_BLUE (Clay_Color) {111, 173, 162, 255}
Clay_TextElementConfig headerTextConfig = { .textColor = {0,0,0,255}, .fontId = 0, .fontSize = 16, .letterSpacing = 5 };

using namespace std::chrono_literals;

void HandleClayErrors(Clay_ErrorData errorData)
{
    RCLCPP_ERROR(rclcpp::get_logger("clay"), "%s", errorData.errorText.chars);
}

class IGVCDisplay : public rclcpp::Node
{
public:
    IGVCDisplay() : Node("igvc_display")
    {
        // int
        init();

        // if we reach here our window closed, kill this ros node
        rclcpp::shutdown();
    }

private:
    Clay_RenderCommandArray CreateLayout(void)
    {
        Clay_BeginLayout();

        // create a horizonal header bar
        CLAY_AUTO_ID({ .layout = { .sizing = { .width = CLAY_SIZING_GROW(1), .height = CLAY_SIZING_GROW(0) }, .padding = {8, 8, 8, 8 }, .childGap = 8, .childAlignment = { .x = CLAY_ALIGN_X_RIGHT } }, .backgroundColor =  {180, 180, 180, 255} }) {            
            RenderHeaderButton(CLAY_STRING("Header Item 1"));
            RenderHeaderButton(CLAY_STRING("Header Item 2"));
            RenderHeaderButton(CLAY_STRING("Header Item 3"));
            RenderHeaderButton(CLAY_STRING("Header Item 4"));
            RenderHeaderButton(CLAY_STRING("Header Item 5"));
        }

        return Clay_EndLayout();
    }

    Clay_ElementDeclaration HeaderButtonStyle(bool hovered) {
        return (Clay_ElementDeclaration) {
            .layout = {.sizing = {.width = CLAY_SIZING_GROW(1), .height = CLAY_SIZING_GROW(0)}, .padding = {16, 16, 8, 8}},
            .backgroundColor = hovered ? COLOR_ORANGE : COLOR_BLUE,
        };
    }

    // Examples of re-usable "Components"
    void RenderHeaderButton(Clay_String text) {
        CLAY_AUTO_ID(HeaderButtonStyle(Clay_Hovered())) {
            CLAY_TEXT(text, CLAY_TEXT_CONFIG(headerTextConfig));
        }
    }

    void init()
    {
        Clay_Raylib_Initialize(1024, 768, "IGVC 2026", 0);

        // create clay arena and context
        uint64_t required_mem = Clay_MinMemorySize();
        Clay_Arena arena = Clay_CreateArenaWithCapacityAndMemory(required_mem, malloc(required_mem));
        Clay_Context *clayContext = Clay_Initialize(arena, (Clay_Dimensions){.width = 1024, .height = 768}, (Clay_ErrorHandler){HandleClayErrors});

        // load and create fonts
        Font fonts[1];
        fonts[0] = LoadFontEx("build/igvc_display/resources/Roboto.ttf", 48, 0, 400);
        SetTextureFilter(fonts[0].texture, TEXTURE_FILTER_BILINEAR);
        Clay_SetMeasureTextFunction(Raylib_MeasureText, fonts);

        // set image icon
        Image icon = LoadImage("build/igvc_display/resources/scr.png");
        SetWindowIcon(icon);
        UnloadImage(icon);

        // shader testing
        // Shader exampleShader;
        // int timeLoc;
        // float startTime;
        // exampleShader = LoadShader(0, "build/igvc_display/resources/example_shader.fs");
        // timeLoc = GetShaderLocation(exampleShader, "time");
        // startTime = GetTime();

        // render while window is open and ROS is ok
        while (!WindowShouldClose() && rclcpp::ok())
        {
            BeginDrawing();

            Clay_RenderCommandArray renderCommands = CreateLayout();

            ClearBackground(BLACK);

            // BeginShaderMode(exampleShader);

            // Update shader uniform values
            // float timeValue = GetTime() - startTime;
            // SetShaderValue(exampleShader, timeLoc, &timeValue, SHADER_UNIFORM_FLOAT);

            // Mouse Stuff
            Clay_Vector2 mousePosition = RAYLIB_VECTOR2_TO_CLAY_VECTOR2(GetMousePosition());
            Clay_SetPointerState(mousePosition, IsMouseButtonDown(0));
            Clay_SetLayoutDimensions((Clay_Dimensions) { (float)GetScreenWidth(), (float)GetScreenHeight() });

            Clay_Raylib_Render(renderCommands, fonts);

            // EndShaderMode();

            EndDrawing();
        }

        Clay_Raylib_Close();
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    std::shared_ptr<IGVCDisplay> node = std::make_shared<IGVCDisplay>();
    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}