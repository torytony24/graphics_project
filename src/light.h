#pragma once
#ifndef LIGHT_H
#define LIGHT_H

#include <glad/glad.h>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

#include <vector>

class DirectionalLight {
public:
	float azimuth;
	float elevation;
	glm::vec3 lightDir; // direction of light. If elevation is 90, it would be (0,-1,0)
	glm::vec3 lightColor; // this is I_d (I_s = I_d, I_a = 0.3 * I_d)

	// ctor: azimuth/elevation (degrees) -> compute lightDir
	DirectionalLight(float azimuth_, float elevation_, glm::vec3 lightColor_)
		: azimuth(azimuth_), elevation(elevation_), lightDir(0.0f, -1.0f, 0.0f), lightColor(lightColor_)
	{
		updateLightDir();
	}

	// ctor: supply lightDir directly -> compute azimuth/elevation for consistency
	DirectionalLight(glm::vec3 lightDir_, glm::vec3 lightColor_)
		: azimuth(0.0f), elevation(45.0f), lightDir(glm::normalize(lightDir_)), lightColor(lightColor_)
	{
		// compute elevation: updateLightDir assumes lightDir.y = -sin(elevation)
		// clamp for safety
		float sy = glm::clamp(-lightDir.y, -1.0f, 1.0f);
		elevation = glm::degrees(asin(sy));
		// azimuth from x,z
		azimuth = glm::degrees(atan2(lightDir.z, lightDir.x));
	}

	glm::mat4 getViewMatrix(glm::vec3 cameraPosition) {
		// directional light has no light position. Assume fake light position depending on camera position.
		float lightDistance = 15.0f;
		glm::vec3 lightPos = cameraPosition - this->lightDir * lightDistance;
		return glm::lookAt(lightPos, cameraPosition, glm::vec3(0, 1, 0));
	}

	glm::mat4 getProjectionMatrix() {
		// For simplicity, just use static projection matrix. (Actually we have to be more accurate with considering camera's frustum)
		return glm::ortho(-15.0f, 15.0f, -15.0f, 15.0f, 0.1f, 50.0f);
	}

	void updateLightDir() {
		// TODO:
		float radAzimuth = glm::radians(azimuth);
		float radElevation = glm::radians(elevation);

		lightDir.x = cos(radElevation) * cos(radAzimuth);
		lightDir.y = -sin(radElevation);
		lightDir.z = cos(radElevation) * sin(radAzimuth);

		lightDir = glm::normalize(lightDir);
	}

	// Processes input received from a mouse input system. Expects the offset value in both the x(azimuth) and y(elevation) direction.
	void processKeyboard(float xoffset, float yoffset)
	{
		azimuth += xoffset;
		elevation += yoffset;

		if (elevation > 80.0f) elevation = 80.0f;
		if (elevation < 15.0f) elevation = 15.0f;

		updateLightDir();
	}
};

#endif