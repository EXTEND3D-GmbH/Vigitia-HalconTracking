#pragma once
#include <memory>
#include <string>
#include <opencv2/opencv.hpp>

void Initialize();

struct Rect
{
	double xMin;
	double xMax;
	double yMin;
	double yMax;
};

struct ShapeModelResult
{
	cv::Point2d pos;
	double score;
	bool inSearchRegion;

	struct BoundingCircle
	{
		cv::Point2d pos;
		double radius;

		bool intersects( const BoundingCircle& other ) const
		{
			auto p = pos - other.pos;
			return sqrt( p.x * p.x + p.y * p.y ) <= ( radius + other.radius );
		}

	} circle;
};

enum class SearchMode
{
	Table,
	SearchRegion
};

class HalconImage
{
	friend class ShapeModelDetector;

public:
	explicit HalconImage( cv::Mat image );
	~HalconImage();

private:
	class Impl;
	std::unique_ptr< Impl > m_pimpl;
};

class ShapeModelDetector
{
public:
	ShapeModelDetector();
	ShapeModelDetector( Rect table );
	~ShapeModelDetector();	// defined in the implementation file, where impl is a complete type
	ShapeModelDetector( ShapeModelDetector&& ) noexcept;  // defined in the implementation file
	ShapeModelDetector( const ShapeModelDetector& ) = delete;
	ShapeModelDetector& operator=( ShapeModelDetector&& ) noexcept;	 // defined in the implementation file
	ShapeModelDetector& operator=( const ShapeModelDetector& ) = delete;

	void setImage( HalconImage& image );
	void findRegionROIFromImage( int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax );
	void addAutoShapeModel( double minScore,
							int hueMin,
							int hueMax,
							int satMin,
							int satMax,
							int valMin,
							int valMax,
							int dilationSize,
							int minContrast );
	std::vector< ShapeModelResult > findShapeModel( double minScore,
													int maxMatches,
													SearchMode mode,
													bool hueCheck ) const;
	bool serialize( const std::string& folderPath );
	bool deserialize( const std::string& folderPath );

	void debugShowHueExtract(
		int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax, int dilationSize );

	size_t modelCount() const;
	static void showImage( const HalconImage& image );

private:
	class Impl;
	std::unique_ptr< Impl > m_pimpl;
};