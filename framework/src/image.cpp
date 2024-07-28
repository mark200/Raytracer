#include "image.h"
// Suppress warnings in third-party code.
#include <framework/disable_all_warnings.h>
DISABLE_WARNINGS_PUSH()
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
DISABLE_WARNINGS_POP()
#include <cassert>
#include <exception>
#include <iostream>
#include <string>

Image::Image(const std::filesystem::path& filePath)
{
    if (!std::filesystem::exists(filePath)) {
        std::cerr << "Texture file " << filePath << " does not exists!" << std::endl;
        throw std::exception();
    }

    const auto filePathStr = filePath.string(); // Create l-value so c_str() is safe.
    int numChannels;
    stbi_uc* pixels = stbi_load(filePathStr.c_str(), &m_width, &m_height, &numChannels, STBI_rgb);

    if (numChannels < 3) {
        std::cerr << "Only textures with 3 or more color channels are supported. " << filePath << " has " << numChannels << " channels" << std::endl;
        throw std::exception();
    }
    if (!pixels) {
        std::cerr << "Failed to read texture " << filePath << " using stb_image.h" << std::endl;
        throw std::exception();
    }

    //std::cout << "Number of channels in texture: " << numChannels << std::endl;
    for (size_t i = 0; i < m_width * m_height * numChannels; i += numChannels) {
        m_pixels.emplace_back(pixels[i + 0] / 255.0f, pixels[i + 1] / 255.0f, pixels[i + 2] / 255.0f);
    }

    stbi_image_free(pixels);
}

glm::vec3 Image::getTexel(const glm::vec2& textureCoordinates) const
{
    // TODO: read the correct pixel from m_pixels.
    // The pixels are stored in row major order.

    int partX = textureCoordinates.x;
    float boxX = textureCoordinates.x - partX;
    int partY = 1 - textureCoordinates.y;
    float boxY = (1 - textureCoordinates.y) - partY;
    
    float divX = 1.0 / m_width;
    float divY = 1.0 / m_height;
    int pixelX = boxX / divX; 
    int pixelY = boxY / divY; 
    int pixel = std::min(pixelX, m_width - 1) + std::min(pixelY, m_height - 1) * m_width;
    return m_pixels[pixel];
    
    // else
    // const glm::ivec2 pixel = glm::ivec2(textureCoordinates * glm::vec2(m_width, m_height) + 0.5f);
    // return m_pixels[pixel.y * m_width + pixel.x];
}
