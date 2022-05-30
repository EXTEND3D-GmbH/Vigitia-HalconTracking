#pragma once
#include <GL/glew.h>
#include <opencv2/opencv.hpp>
class Texture
{
public:
	Texture()
	{
		glGenTextures( 1, &m_texture );
		glBindTexture( GL_TEXTURE_2D, m_texture );

		// Setup filtering parameters for display
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR );
		glTexParameteri( GL_TEXTURE_2D,
						 GL_TEXTURE_WRAP_S,
						 GL_CLAMP_TO_EDGE );  // This is required on WebGL for non power-of-two textures
		glTexParameteri( GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE );	// Same
	}

	~Texture()
	{
		glDeleteTextures( 1, &m_texture );
	}

	void update( cv::Mat mat )
	{
		if ( mat.empty() ) return;

		glBindTexture( GL_TEXTURE_2D, m_texture );
		glTexImage2D( GL_TEXTURE_2D, 0, GL_RGB, mat.cols, mat.rows, 0, GL_BGRA, GL_UNSIGNED_BYTE, mat.data );
	}

	GLuint texture() const
	{
		return m_texture;
	}

	Texture( const Texture& ) = delete;
	Texture operator=( const Texture& ) = delete;

private:
	GLuint m_texture;
};

cv::Mat colorizeDepthImage( cv::Mat input, float rangeStart, float rangeEnd )
{
	cv::Mat range;
	cv::min( cv::max( input, rangeStart ), rangeEnd, range );
	cv::Mat out = ( range - rangeStart );
	cv::Mat out2;
	out.convertTo( out2, CV_32F, 1 / ( rangeEnd - rangeStart ) );
	cv::Mat normalized;
	out2.convertTo( normalized, CV_8U, 255 );
	cv::Mat dst;
	cv::applyColorMap( normalized, dst, cv::COLORMAP_JET );
	return dst;
}