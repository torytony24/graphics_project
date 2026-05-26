#ifndef TEXTURE_H
#define TEXTURE_H
#define STB_IMAGE_IMPLEMENTATION   // use of stb functions once and for all
#include "stb_image.h"

#include <string>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include <algorithm>
#include <glad/glad.h>
#include <GLFW/glfw3.h>
class Texture{
public:
    unsigned int ID;
    int width;
    int height;
    int channels;
    Texture() {}
    Texture(const char* filePath)
    {   
        glGenTextures(1, &ID);
        glBindTexture(GL_TEXTURE_2D, ID);
        // set the texture wrapping parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);	// set texture wrapping to GL_REPEAT (default wrapping method)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        // set texture filtering parameters
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        stbi_set_flip_vertically_on_load(true);
        
        unsigned char *data = stbi_load(filePath, &width, &height, &channels, 0);
        if (data)
        {   
            if(channels == 3){
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
            }else{
                glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);
            }
            
            glGenerateMipmap(GL_TEXTURE_2D);
        }
        else
        {
            std::cout << "Failed to load texture" << std::endl;
        }
        stbi_image_free(data);
    }
};

class ThinFilmLUTTexture
{
public:
    unsigned int ID;
    int width;
    int height;

    ThinFilmLUTTexture(int lutWidth = 256, int lutHeight = 128)
        : ID(0), width(lutWidth), height(lutHeight)
    {
        // Precompute the LUT on the CPU so the shader can sample interference colors cheaply.
        std::vector<unsigned char> pixels(width * height * 3);

        for (int y = 0; y < height; ++y) {
            float cosTheta = static_cast<float>(y) / static_cast<float>(height - 1);
            for (int x = 0; x < width; ++x) {
                float deltaNm = 1200.0f * static_cast<float>(x) / static_cast<float>(width - 1);
                float angleBoost = 0.58f + 0.42f * cosTheta;
                float optical = deltaNm * angleBoost;

                float r = 0.5f + 0.5f * std::cos(6.2831853f * optical / 680.0f + 0.0f);
                float g = 0.5f + 0.5f * std::cos(6.2831853f * optical / 540.0f + 2.1f);
                float b = 0.5f + 0.5f * std::cos(6.2831853f * optical / 440.0f + 4.2f);

                float whitening = 0.22f + 0.18f * (1.0f - cosTheta);
                r = r * (1.0f - whitening) + whitening;
                g = g * (1.0f - whitening) + whitening;
                b = b * (1.0f - whitening) + whitening;

                int idx = (y * width + x) * 3;
                pixels[idx + 0] = static_cast<unsigned char>(std::max(0.0f, std::min(1.0f, r)) * 255.0f);
                pixels[idx + 1] = static_cast<unsigned char>(std::max(0.0f, std::min(1.0f, g)) * 255.0f);
                pixels[idx + 2] = static_cast<unsigned char>(std::max(0.0f, std::min(1.0f, b)) * 255.0f);
            }
        }

        glGenTextures(1, &ID);
        glBindTexture(GL_TEXTURE_2D, ID);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, &pixels[0]);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glBindTexture(GL_TEXTURE_2D, 0);
    }
};

class DepthMapTexture 
{
public:
	unsigned int ID;
	unsigned int depthMapFBO;
	int width;
	int height;

	DepthMapTexture(int shadow_width, int shadow_height) 
	{
		width = shadow_width;
		height = shadow_height;
		glGenFramebuffers(1, &depthMapFBO);
		glGenTextures(1, &ID);
		glBindTexture(GL_TEXTURE_2D, ID);
		glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT, width, height, 0, GL_DEPTH_COMPONENT, GL_FLOAT, NULL);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_BORDER);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_BORDER);
		float borderColor[] = { 1.0, 1.0, 1.0, 1.0 };
		glTexParameterfv(GL_TEXTURE_2D, GL_TEXTURE_BORDER_COLOR, borderColor);
		// attach depth texture as FBO's depth buffer
		glBindFramebuffer(GL_FRAMEBUFFER, depthMapFBO);
		glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, ID, 0);
		glDrawBuffer(GL_NONE);
		glReadBuffer(GL_NONE);
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
	}
};
#endif
