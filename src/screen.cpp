#include "screen.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#include <glm/common.hpp>
#include <glm/vec3.hpp>
#include <glm/vec4.hpp>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>
DISABLE_WARNINGS_POP()
#include <algorithm>
#include <iostream>
#include <framework/opengl_includes.h>
#include <string>

Screen::Screen(const glm::ivec2& resolution)
    : m_resolution(resolution)
    , m_textureData(size_t(resolution.x * resolution.y), glm::vec3(0.0f))
{
    // Generate texture
    glGenTextures(1, &m_texture);
    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glBindTexture(GL_TEXTURE_2D, 0);
}

void Screen::clear(const glm::vec3& color)
{
    std::fill(std::begin(m_textureData), std::end(m_textureData), color);
}

void Screen::setPixel(int x, int y, const glm::vec3& color)
{
    // In the window/camera class we use (0, 0) at the bottom left corner of the screen (as used by GLFW).
    // OpenGL / stbi like the origin / (-1,-1) to be at the TOP left corner so transform the y coordinate.
    const int i = (m_resolution.y - 1 - y) * m_resolution.x + x;
    m_textureData[i] = glm::vec4(color, 1.0f);
}

void Screen::writeBitmapToFile(const std::filesystem::path& filePath)
{
    std::vector<glm::u8vec4> textureData8Bits(m_textureData.size());
    std::transform(std::begin(m_textureData), std::end(m_textureData), std::begin(textureData8Bits),
        [](const glm::vec3& color) {
            const glm::vec3 clampedColor = glm::clamp(color, 0.0f, 1.0f);
            return glm::u8vec4(glm::vec4(clampedColor, 1.0f) * 255.0f);
        });

    std::string filePathString = filePath.string();
    stbi_write_bmp(filePathString.c_str(), m_resolution.x, m_resolution.y, 4, textureData8Bits.data());
}

void Screen::draw()
{
    glPushAttrib(GL_ALL_ATTRIB_BITS);

    glBindTexture(GL_TEXTURE_2D, m_texture);
    glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB32F, m_resolution.x, m_resolution.y, 0, GL_RGB, GL_FLOAT, m_textureData.data());

    glDisable(GL_LIGHTING);
    glDisable(GL_LIGHT0);
    glDisable(GL_COLOR_MATERIAL);
    glDisable(GL_NORMALIZE);
    glColor3f(1.0f, 1.0f, 1.0f);

    glEnable(GL_TEXTURE_2D);
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_texture);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();

    glBegin(GL_QUADS);
    glTexCoord2f(0.0f, 1.0f);
    glVertex3f(-1.0f, -1.0f, 0.0f);
    glTexCoord2f(1.0f, 1.0f);
    glVertex3f(+1.0f, -1.0f, 0.0f);
    glTexCoord2f(1.0f, 0.0f);
    glVertex3f(+1.0f, +1.0f, 0.0f);
    glTexCoord2f(0.0f, 0.0f);
    glVertex3f(-1.0f, +1.0f, 0.0f);
    glEnd();

    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glPopMatrix();

    glPopAttrib();
}

// The main filtering method
void Screen::filterImage(int filterSize, float threshold, float scale) {

    std::vector<glm::vec3> copyThreshold;

    // First copy the image
    for (int i = 0; i < m_textureData.size(); i++)
        copyThreshold.push_back(glm::vec3{this->m_textureData[i].x, this->m_textureData[i].y, this->m_textureData[i].z});

    // Extract large values
    for (int i = 0; i < m_textureData.size(); i++) {
        if (copyThreshold[i].x < threshold && copyThreshold[i].y < threshold && copyThreshold[i].z < threshold) {
            copyThreshold[i].x = 0.0f;
            copyThreshold[i].y = 0.0f;
            copyThreshold[i].z = 0.0f;
        }
    }

    // Box filter on the copy
    for (int i = filterSize; i < this->m_resolution.x - filterSize; i++)
        for (int j = filterSize; j < this->m_resolution.y - filterSize; j++) {
            copyThreshold[j * this->m_resolution.y + i].x = boxFilter(copyThreshold, i, j, 0, filterSize);
            copyThreshold[j * this->m_resolution.y + i].y = boxFilter(copyThreshold, i, j, 1, filterSize);
            copyThreshold[j * this->m_resolution.y + i].z = boxFilter(copyThreshold, i, j, 2, filterSize);
        }

    // Scale box-filtered copy
    for (int i = 0; i < m_textureData.size(); i++) {
        copyThreshold[i].x *= scale;
        copyThreshold[i].y *= scale;
        copyThreshold[i].z *= scale;
    }

    // Add the copy to the original image
    for (int i = 0; i < m_textureData.size(); i++) {
        m_textureData[i].x += copyThreshold[i].x;
        m_textureData[i].y += copyThreshold[i].y;
        m_textureData[i].z += copyThreshold[i].z;
    }
}

float Screen::getPixel(std::vector<glm::vec3>& pixels, int i, int j, int col) {
    if (col == 0)
        return pixels[j * this->m_resolution.y + i].x;
    else if (col == 1)
        return pixels[j * this->m_resolution.y + i].y;
    else if (col == 2)
        return pixels[j * this->m_resolution.y + i].z;
}

float Screen::boxFilter(std::vector<glm::vec3>& pixels, int i, int j, int col, int filterSize) {
    float sum = 0.0f;

    for (int x = -filterSize; x < filterSize + 1; x++)
        for (int y = -filterSize; y < filterSize + 1; y++)
            sum += getPixel(pixels, i + x, j + y, col);

    sum /= (2 * filterSize + 1) * (2 * filterSize + 1);
    return sum;
}