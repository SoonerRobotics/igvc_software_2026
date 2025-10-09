#version 330

in vec2 fragTexCoord;
in vec4 fragColor;

out vec4 finalColor;
uniform float time;

void main()
{
    float r = 0.5 + 0.5 * sin(time + fragTexCoord.x * 10.0);
    float g = 0.5 + 0.5 * sin(time + fragTexCoord.y * 10.0);
    float b = 0.5 + 0.5 * sin(time);

    finalColor = vec4(r, g, b, 1.0) * fragColor;
}