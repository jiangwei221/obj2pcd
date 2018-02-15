//based on opengl-tutorial.org

#ifndef OBJLOADER_H
#define OBJLOADER_H

#include <vector>
#include <iostream>
#include <stdio.h>
#include "glm/vec2.hpp"
#include "glm/vec3.hpp"

using namespace glm;

bool loadOBJ(const char *path, std::vector<dvec3> &out_vertices, std::vector<dvec3> &out_normals)
{
	printf("Loading OBJ file %s...\n", path);

	std::vector<unsigned int> vertexIndices, normalIndices;
	std::vector<dvec3> temp_vertices;
	std::vector<dvec3> temp_normals;

	FILE *file = fopen(path, "r");
	if (file == NULL)
	{
		printf("Impossible to open the file ! Are you in the right path ? See Tutorial 1 for details\n");
		getchar();
		return false;
	}

	while (1)
	{

		char lineHeader[128];
		// read the first word of the line
		int res = fscanf(file, "%s", lineHeader);
		if (res == EOF)
			break; // EOF = End Of File. Quit the loop.

		// else : parse lineHeader

		if (strcmp(lineHeader, "v") == 0)
		{
			dvec3 vertex;
			fscanf(file, "%lf %lf %lf\n", &vertex.x, &vertex.y, &vertex.z);
			temp_vertices.push_back(vertex);
		}
		else if (strcmp(lineHeader, "vn") == 0)
		{
			dvec3 normal;
			fscanf(file, "%lf %lf %lf\n", &normal.x, &normal.y, &normal.z);
			temp_normals.push_back(normal);
		}
		else if (strcmp(lineHeader, "f") == 0)
		{
			char strvertex[3][128];
			int slash_counter[3] = {0, 0, 0};
			bool sample_normal = true;
			int matches = 0;

			//read vertex data tp char array
			matches = fscanf(file, "%s %s %s\n", strvertex[0], strvertex[1], strvertex[2]);
			if (matches != 3)
			{
				printf("File can't be read by our simple parser :-( Try exporting with other options\n");
				fclose(file);
				return false;
			}
			//check the normal data by the number of slash
			//0 or 1 slash means no normal data
			for (int i = 0; i < 3; i++)
			{
				for (int j = 0; j < strlen(strvertex[i]); j++)
				{
					if (strvertex[i][j] == '/')
						slash_counter[i]++;
				}
			}
			for (int i = 0; i < 3; i++)
			{
				if (slash_counter[i] == 1 || slash_counter[i] == 0)
				{
					//no normal data in obj
					sample_normal = false;
					break;
				}
			}

			//if sample_normal == false
			//then push the 3 first int values into vertexIndices
			if (sample_normal == false)
			{
				for (int i = 0; i < 3; i++)
				{
					char *v_id;
					int index;
					v_id = strtok(strvertex[i], "/");
					index = atoi(v_id);
					vertexIndices.push_back(index);
				}
			}
			//if sample_normal == true
			else
			{
				for (int i = 0; i < 3; i++)
				{
					char *content = strdup(strvertex[i]);
					char *v_id;
					int index;
					v_id = strsep(&content, "/");
					index = atoi(v_id);
					vertexIndices.push_back(index);
					v_id = strsep(&content, "/");
					v_id = strsep(&content, "/");
					index = atoi(v_id);
					normalIndices.push_back(index);
				}
			}
		}
		else
		{
			// Probably a comment, eat up the rest of the line
			char stupidBuffer[1000];
			fgets(stupidBuffer, 1000, file);
		}
	}
	std::cout << "vertexIndices: " << vertexIndices.size() << std::endl;
	std::cout << "normalIndices: " << normalIndices.size() << std::endl;
	// For each vertex of each triangle
	for (unsigned int i = 0; i < vertexIndices.size(); i++)
	{

		// Get the indices of its attributes
		unsigned int vertexIndex = vertexIndices[i];

		// Get the attributes thanks to the index
		vec3 vertex = temp_vertices[vertexIndex - 1];

		// Put the attributes in buffers
		out_vertices.push_back(vertex);
	}
	for (unsigned int i = 0; i < normalIndices.size(); i++)
	{

		// Get the indices of its attributes
		unsigned int normalIndex = normalIndices[i];

		// Get the attributes thanks to the index
		vec3 normal = temp_normals[normalIndex - 1];

		// Put the attributes in buffers
		out_normals.push_back(normal);
	}
	fclose(file);
	return true;
}

#endif