#include "ShapeModel.h"

#include <HalconCpp.h>
#include <nlohmann/json.hpp>

#include <vector>
#include <optional>
#include <filesystem>
#include <fstream>

using namespace HalconCpp;

static std::unique_ptr< HWindow > window;
void Initialize()
{
	HSystem::ResetObjDb( 1920, 1080, 3 );
	SetSystem( "use_window_thread", "true" );
	window = std::make_unique< HWindow >( 16, 16, 1280, 720, 0, "visible", "" );
}

struct WindowFlusher
{
	WindowFlusher( HWindow* w ) : window( w )
	{
		if ( window ) {
			window->SetWindowParam( "flush", "false" );
		}
	}

	~WindowFlusher()
	{
		if ( window ) {
			window->SetWindowParam( "flush", "true" );
			window->FlushBuffer();
		}
	}
	HWindow* window;
};

static HImage halconImageFromMat( cv::Mat image )
{
	// Only support 1/3 Channel Images for now
	const auto nChannels = image.channels();
	if ( nChannels != 1 && nChannels != 3 && nChannels != 4 ) {
		throw std::exception( "Unsupported amount of channels in to halcon conversion" );
	}

	auto bitsToString = []( int bitDepth ) {
		switch ( bitDepth ) {
			case 8:
				return "byte";
			case 16:
				return "uint2";
			case 32:
				// Assuming float here
				return "real";
			default:
				throw std::exception( "Unsupported bitDepth in halcon image conversion" );
		}
	};

	// Could avoid the copy by using GenImageXExtern but how / when do we free?
	// Furthermore this does not work with N-Channels images as they need to be given In a per channel format

	HImage result;
	if ( nChannels == 1 ) {
		result.GenImage1( bitsToString( image.depth() ), image.cols, image.rows, image.data );
	} else {
		// This is bad, we need to extract the channels which incurs 1 copy
		// and then copy into the image since we cant track the image for now which is another copy :(

		std::vector< cv::Mat > channels;
		cv::split( image, channels );
		assert( channels.size() == 3 );

		// Clone the image to make it contiguous
		cv::Mat channelR = channels[ 0 ].clone();
		cv::Mat channelG = channels[ 1 ].clone();
		cv::Mat channelB = channels[ 2 ].clone();

		assert( channelR.isContinuous() );
		assert( channelG.isContinuous() );
		assert( channelB.isContinuous() );
		result.GenImage3( bitsToString( image.depth() ),
						  image.cols,
						  image.rows,
						  channelR.data,
						  channelG.data,
						  channelB.data );
	}
	return result;
}

class HalconImage::Impl
{
public:
	Impl( cv::Mat image )
	{
		hImage = halconImageFromMat( image );
	}

	HImage hImage;
};

class ShapeModelDetector::Impl
{
public:
	Impl( Rect table ) : tableRoi( table.yMin, table.xMin, table.yMax, table.xMax )
	{}

	void setImage( HalconImage::Impl& image )
	{
		hImage = image.hImage;
		tableImage = hImage.ReduceDomain( tableRoi );

		HRegion reduction = tableRoi;
		if ( searchRoi.has_value() ) {
			reduction = reduction.Intersection( searchRoi.value() );
		}
		searchImage = hImage.ReduceDomain( reduction );
	}

	void findRegionROIFromImage( int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax )
	{
		auto regions = findHSVRegion( tableImage, hueMin, hueMax, satMin, satMax, valMin, valMax );
		auto filledRegion = regions
								.FillUp()						  // Fill regions
								.ErosionRectangle1( 5, 5 )		  // Erode for better edges
								.Connection()					  // Connectedcomponent
								.SelectShapeStd( "max_area", 0 )  // Biggest region
								.DilationRectangle1( 10, 10 );

		searchRoi = filledRegion;

		if ( window ) {
			int width = tableImage.Width()[ 0 ];
			int height = tableImage.Height()[ 0 ];
			window->SetPart( 0, 0, height - 1, width - 1 );
			window->DispColor( tableImage );
			window->SetColor( "gray" );
			window->DispRegion( tableRoi );
			window->SetColor( "green" );
			window->DispRegion( searchRoi.value() );
		}
	}

	void debugShowHueExtract(
		int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax, int dilationSize )
	{
		WindowFlusher w( window.get() );

		auto roiImage = searchImage;
		auto objImage
			= extractObject( roiImage, hueMin, hueMax, satMin, satMax, valMin, valMax, dilationSize );

		if ( window ) {
			int width = objImage.Width()[ 0 ];
			int height = objImage.Height()[ 0 ];
			window->SetPart( 0, 0, height - 1, width - 1 );
			window->SetColor( "red" );
			window->DispRegion( objImage );
		}
	}

	void addAutoShapeModel( double minScore,
							int hueMin,
							int hueMax,
							int satMin,
							int satMax,
							int valMin,
							int valMax,
							int dilationSize,
							int minContrast )
	{
		WindowFlusher w( window.get() );

		auto roiImage = searchImage;
		auto objImage
			= extractObject( roiImage, hueMin, hueMax, satMin, satMax, valMin, valMax, dilationSize );

		try {
			// Magic 11 from HDevelop example for contrast
			HShapeModel model(
				objImage, 6, 0.0, 2 * PI, "auto", "auto", "ignore_color_polarity", "auto", minContrast );
			m_hueMin = hueMin;
			m_hueMax = hueMax;
			m_satMin = satMin;
			m_satMax = satMax;
			m_valMin = valMin;
			m_valMax = valMax;
			shapeModels.push_back( model );

			if ( window ) {
				HTuple row, column, angle, score, m;
				roiImage.FindShapeModels( model,
										  -PI / 2,
										  PI / 2,
										  0.8,
										  1,
										  0,
										  "least_squares",
										  0,
										  0.9,
										  &row,
										  &column,
										  &angle,
										  &score,
										  &m );

				if ( m.Length() > 0 ) {
					displayShapeMatch( model, angle.D(), row.D(), column.D() );
				}
			}

		} catch ( HException e ) {
			printf( "%s\n", e.ErrorMessage().Text() );
		}
	}

	std::vector< ShapeModelResult > findShapeModel( double minScore,
													int maxMatches,
													SearchMode mode,
													bool hueCheck ) const
	{
		if ( minScore < 0.1 ) {
			return {};
		}
		WindowFlusher w( window.get() );
		if ( shapeModels.empty() ) {
			return {};
		}

		HImage roiImage;
		switch ( mode ) {
			case SearchMode::Table: {
				roiImage = tableImage;
				break;
			}
			case SearchMode::SearchRegion: {
				roiImage = searchImage;
				break;
			}
		}

		// Reduce search area by hue check colors
		HImage C2, C3, S, V;
		auto C1 = roiImage.Decompose3( &C2, &C3 );
		auto H = C1.TransFromRgb( C2, C3, &S, &V, "hsv" );

		if ( hueCheck ) {
			auto region
				= findHSVRegion( roiImage, m_hueMin, m_hueMax, m_satMin, m_satMax, m_valMin, m_valMax );
			roiImage = roiImage.ReduceDomain( region );
		}

		HTuple shapeModelTuple;
		for ( const auto& model : shapeModels ) {
			shapeModelTuple.Append( model );
		}
		HShapeModelArray shapeArray;
		shapeArray.SetFromTuple( shapeModelTuple );

		HTuple params;
		params.Append( "least_squares" );
		params.Append( "max_deformation 4" );
		HTuple row, column, angle, score, model;
		roiImage.FindShapeModels( shapeArray,
								  -PI / 2,
								  PI / 2,
								  minScore,
								  maxMatches,
								  0,
								  params,
								  0,
								  0.9,
								  &row,
								  &column,
								  &angle,
								  &score,
								  &model );

		std::vector< ShapeModelResult > results;
		for ( Hlong i = 0; i < model.Length(); i++ ) {
			auto& m = shapeModels[ model[ i ].L() ];
			auto r = row[ i ].D();
			auto c = column[ i ].D();
			if ( hueCheck ) {
				auto checkHisto = []( HImage image, HRegion region, int min, int max ) {
					HTuple histo;
					image.GrayHisto( region, &histo );
					double sum = 0.0;
					if ( min < max ) {
						for ( size_t i = min; i <= max; i++ ) {
							sum += histo[ i ].D();
						}
					} else {
						for ( size_t i = min; i <= 255; i++ ) {
							sum += histo[ i ].D();
						}
						for ( size_t i = 0; i <= max; i++ ) {
							sum += histo[ i ].D();
						}
					}

					return sum >= 0.7;
				};
				HRegion region( r - 10, c - 10, r + 10, c + 10 );
				if ( !checkHisto( H, region, m_hueMin, m_hueMax ) ) {
					continue;
				}
				if ( !checkHisto( S, region, m_satMin, m_satMax ) ) {
					continue;
				}
				if ( !checkHisto( V, region, m_valMin, m_valMax ) ) {
					continue;
				}
			}

			HHomMat2D mat;
			mat.HomMat2dIdentity();
			mat = mat.HomMat2dRotate( angle[ i ].D(), 0, 0 ).HomMat2dTranslate( r, c );

			bool inSearchRegion = true;
			if ( searchRoi.has_value() ) {
				inSearchRegion = searchRoi->TestRegionPoint( r, c );
			}
			auto& result = results.emplace_back(
				ShapeModelResult{ cv::Point2d( c, r ), score[ i ].D(), inSearchRegion } );

			window->SetColor( "cyan" );
			auto transformed = m.GetShapeModelContours( 1 ).AffineTransContourXld( mat );
			HRegion region;
			region.GenEmptyRegion();
			for ( size_t i = 0; i < transformed.CountObj(); i++ ) {
				auto cont = transformed[ i + 1 ];
				auto regionCont = cont.GenRegionContourXld( "filled" );
				region = region.Union2( regionCont );
			}
			window->DispXld( transformed );
			region.SmallestCircle( &result.circle.pos.y, &result.circle.pos.x, &result.circle.radius );
		}

		return results;
	}

	bool serialize( const std::string& folderPath )
	{
		if ( shapeModels.empty() ) {
			return false;
		}
		std::filesystem::create_directories( folderPath );
		std::filesystem::path folder = folderPath;

		{
			auto r = folder / "TableRegion.hobj";
			tableRoi.WriteRegion( r.c_str() );
		}
		if ( searchRoi.has_value() ) {
			auto r = folder / "SearchRegion.hobj";
			searchRoi.value().WriteRegion( r.c_str() );
		}

		int i = 0;
		for ( const auto& model : shapeModels ) {
			std::stringstream ss;
			ss << "ShapeModel_" << i++ << ".sm";
			auto fullPath = ( std::filesystem::path( folderPath ) / ss.str() ).string();
			try {
				model.WriteShapeModel( fullPath.c_str() );
			} catch ( HException e ) {
				printf( "%s\n", e.ErrorMessage().Text() );
			}
		}

		using namespace nlohmann;
		json config;
		config[ "hueMin" ] = m_hueMin;
		config[ "hueMax" ] = m_hueMax;
		config[ "satMin" ] = m_satMin;
		config[ "satMax" ] = m_satMax;
		config[ "valMin" ] = m_valMin;
		config[ "valMax" ] = m_valMax;

		std::ofstream o( folder / "config.json" );
		o << std::setw( 4 ) << config << std::endl;

		return true;
	}

	bool deserialize( const std::string& folderPath )
	{
		std::filesystem::path folder = folderPath;
		{
			auto r = folder / "TableRegion.hobj";
			if ( std::filesystem::is_regular_file( r ) ) {
				tableRoi.ReadRegion( r.c_str() );
			} else {
				return false;
			}
		}

		{
			auto r = folder / "SearchRegion.hobj";
			if ( std::filesystem::is_regular_file( r ) ) {
				HRegion reg;
				reg.ReadRegion( r.c_str() );
				searchRoi = reg;
			}
		}

		{
			using namespace nlohmann;
			json config;

			auto r = folder / "config.json";
			if ( std::filesystem::is_regular_file( r ) ) {
				std::ifstream i( r );
				i >> config;
				m_hueMin = config.value( "hueMin", 0 );
				m_hueMax = config.value( "hueMax", 255 );
				m_satMin = config.value( "satMin", 0 );
				m_satMax = config.value( "satMax", 255 );
				m_valMin = config.value( "valMin", 0 );
				m_valMax = config.value( "valMax", 255 );
			} else {
				return false;
			}
		}

		for ( auto const& entry : std::filesystem::directory_iterator{ folderPath } ) {
			if ( entry.is_regular_file() ) {
				auto path = entry.path();
				auto ext = path.extension().string();
				auto pathStr = path.string();
				if ( ext == ".sm" ) {
					HShapeModel model;
					model.ReadShapeModel( path.c_str() );

					shapeModels.push_back( model );
				}
			}
		}
		return !shapeModels.empty();
	}

	size_t modelCount() const
	{
		return shapeModels.size();
	}

	static void showImage( const HImage& hImage )
	{
		if ( window ) {
			int width = hImage.Width()[ 0 ];
			int height = hImage.Height()[ 0 ];
			window->SetPart( 0, 0, height - 1, width - 1 );
			window->DispColor( hImage );
		}
	}

private:
	HRegion findHSVRegion(
		HImage image, int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax ) const
	{
		HImage C2, C3, S, V;
		auto C1 = image.Decompose3( &C2, &C3 );
		HImage H = C1.TransFromRgb( C2, C3, &S, &V, "hsv" );

		auto filterImage = []( HImage image, int min, int max ) {
			HalconCpp::HRegion threshold;
			if ( min < max ) {
				threshold = image.FastThreshold( min, max, 10 );
			} else {
				auto h1 = image.FastThreshold( min, 255, 10 );
				auto h2 = image.FastThreshold( 0, max, 10 );
				threshold = h1.Union2( h2 );
			}
			return threshold;
		};

		// auto saturationThreshold = S.FastThreshold( 100, 255, 10 );
		// auto saturationRegion = saturationThreshold.Connection().SelectShapeStd( "max_area", 0 );

		// Handle hueMin hueMax to allow warp around
		HalconCpp::HRegion hueThreshold = filterImage( H, hueMin, hueMax );
		HalconCpp::HRegion satThreshold = filterImage( S, satMin, satMax );
		HalconCpp::HRegion valThreshold = filterImage( V, valMin, valMax );

		return hueThreshold.Intersection( satThreshold ).Intersection( valThreshold );
	}

	HImage extractObject( HImage image,
						  int hueMin,
						  int hueMax,
						  int satMin,
						  int satMax,
						  int valMin,
						  int valMax,
						  int dilationSize ) const
	{
		auto region = findHSVRegion( image, hueMin, hueMax, satMin, satMax, valMin, valMax )
						  .ErosionRectangle1( 5, 5 )
						  .Connection()
						  .SelectShapeStd( "max_area", 0 )
						  .DilationRectangle1( dilationSize, dilationSize );

		// return objImage.ReduceDomain( saturationRegion.Intersection( hueRegion ) );
		return image.ReduceDomain( region );
	}

	void displayShapeMatch( HShapeModel model, double angle, double row, double column ) const
	{
		if ( window ) {
			auto contour = model.GetShapeModelContours( 1 );
			HHomMat2D mat;
			mat.HomMat2dIdentity();
			mat = mat.HomMat2dRotate( angle, 0, 0 ).HomMat2dTranslate( row, column );

			window->SetColor( "cyan" );
			auto transformed = contour.AffineTransContourXld( mat );
			window->DispXld( transformed );
		}
	}

	HImage hImage;
	HImage tableImage;
	HImage searchImage;

	std::vector< HShapeModel > shapeModels;
	HRegion tableRoi;
	std::optional< HRegion > searchRoi;

	int m_hueMin = 0;
	int m_hueMax = 255;
	int m_satMin = 0;
	int m_satMax = 255;
	int m_valMin = 0;
	int m_valMax = 255;
};

ShapeModelDetector::ShapeModelDetector() : m_pimpl{ std::make_unique< Impl >( Rect() ) }
{}
ShapeModelDetector::ShapeModelDetector( Rect table ) : m_pimpl{ std::make_unique< Impl >( table ) }
{}
ShapeModelDetector::ShapeModelDetector( ShapeModelDetector&& ) noexcept = default;
ShapeModelDetector::~ShapeModelDetector() = default;
ShapeModelDetector& ShapeModelDetector::operator=( ShapeModelDetector&& ) noexcept = default;

void ShapeModelDetector::setImage( HalconImage& image )
{
	m_pimpl->setImage( *image.m_pimpl );
}
void ShapeModelDetector::findRegionROIFromImage(
	int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax )
{
	m_pimpl->findRegionROIFromImage( hueMin, hueMax, satMin, satMax, valMin, valMax );
}
void ShapeModelDetector::addAutoShapeModel( double minScore,
											int hueMin,
											int hueMax,
											int satMin,
											int satMax,
											int valMin,
											int valMax,
											int dilationSize,
											int minContrast )
{
	m_pimpl->addAutoShapeModel(
		minScore, hueMin, hueMax, satMin, satMax, valMin, valMax, dilationSize, minContrast );
}
std::vector< ShapeModelResult > ShapeModelDetector::findShapeModel( double minScore,
																	int maxMatches,
																	SearchMode mode,
																	bool hueCheck ) const
{
	return m_pimpl->findShapeModel( minScore, maxMatches, mode, hueCheck );
}
bool ShapeModelDetector::serialize( const std::string& folderPath )
{
	return m_pimpl->serialize( folderPath );
}
bool ShapeModelDetector::deserialize( const std::string& folderPath )
{
	return m_pimpl->deserialize( folderPath );
}

void ShapeModelDetector::debugShowHueExtract(
	int hueMin, int hueMax, int satMin, int satMax, int valMin, int valMax, int dilationSize )
{
	m_pimpl->debugShowHueExtract( hueMin, hueMax, satMin, satMax, valMin, valMax, dilationSize );
}
size_t ShapeModelDetector::modelCount() const
{
	return m_pimpl->modelCount();
}

void ShapeModelDetector::showImage( const HalconImage& image )
{
	ShapeModelDetector::Impl::showImage( image.m_pimpl->hImage );
}

HalconImage::HalconImage( cv::Mat image ) : m_pimpl( std::make_unique< Impl >( image ) )
{}
HalconImage::~HalconImage() = default;