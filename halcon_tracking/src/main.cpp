#include <interfaces/srv/change_projection.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "ShapeModel.h"
#include "texture.h"

#include "SoundPlayer.h"

#include <algorithm>
#include <execution>
#include <filesystem>
#include <iostream>

// Need this weird include because ROS also comes with an IMGUI version
#include "3rd/imgui/imgui.h"
#include "3rd/imgui/imgui_impl_glfw.h"
#include "3rd/imgui/imgui_impl_opengl3.h"

#include <ImGuiFileDialog.h>

#include "ImGuiFont.h"
#include <nlohmann/json.hpp>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include <thread>

static void glfw_error_callback( int error, const char* description )
{
	printf( "Glfw Error %d: %s\n", error, description );
}

enum class Mode
{
	TEACH,
	RUN
};

// Internal State
static SoundPlayer soundPlayer;
static Mode mode = Mode::RUN;
static bool isImageNewThisFrame = false;

// TEACH State
static int hMin = 0, hMax = 255, vMin = 0, vMax = 255, sMin = 0, sMax = 255;
static std::string baseSavePath = "c:/Data/Vigitia/Degginger/";
static bool learning = false;
static std::unique_ptr< ShapeModelDetector > currentShapeModel;
static cv::Mat lastImage;
static std::unique_ptr< Texture > lastImageTexture;
static std::unique_ptr< Texture > hsvTexture;

// ROS State
using ServiceCall = interfaces::srv::ChangeProjection;

static rclcpp::Node::SharedPtr node;
static std::shared_ptr< rclcpp::Publisher< geometry_msgs::msg::PointStamped > > publisher;
static rclcpp::Client< ServiceCall >::SharedPtr projectionClient;

static std::mutex serviceMutex;
static ServiceCall::Request::SharedPtr currentRunProjectionRequest;
static ServiceCall::Request::SharedPtr lastRunProjectionRequest;
static ServiceCall::Response::SharedPtr lastRunProjectionResult;
static bool rosServiceWorkerRunning = true;
static std::thread rosServiceWorkerThread;

struct GlobalSettings
{
	inline static Rect tableRect{ 400, 1400, 240, 800 };
	inline static double scoreThreshold = 0.8;
	inline static int dilationSize = 11;
	inline static int minContrast = 50;
	inline static std::chrono::seconds fireRespawnTime{ 20 };
	inline static std::chrono::seconds burnRepeatDelay{ 10 };
	inline static std::chrono::seconds stopTimeout{ 30 };

	static void from_json( const nlohmann::json& j )
	{
		tableRect.xMin = j.value( "xMin", tableRect.xMin );
		tableRect.xMax = j.value( "xMax", tableRect.xMax );
		tableRect.yMin = j.value( "yMin", tableRect.yMin );
		tableRect.yMax = j.value( "yMax", tableRect.yMax );

		scoreThreshold = j.value( "scoreThreshold", scoreThreshold );
		dilationSize = j.value( "dilationSize", dilationSize );
		minContrast = j.value( "minContrast", minContrast );

		fireRespawnTime = std::chrono::seconds( j.value( "fireRespawnTime", fireRespawnTime.count() ) );
		burnRepeatDelay = std::chrono::seconds( j.value( "burnRepeatDelay", burnRepeatDelay.count() ) );
		stopTimeout = std::chrono::seconds( j.value( "stopTimeout", stopTimeout.count() ) );
	}

	static void to_json( nlohmann::json& j )
	{
		j[ "xMin" ] = tableRect.xMin;
		j[ "xMax" ] = tableRect.xMax;
		j[ "yMin" ] = tableRect.yMin;
		j[ "yMax" ] = tableRect.yMax;

		j[ "scoreThreshold" ] = scoreThreshold;
		j[ "dilationSize" ] = dilationSize;
		j[ "minContrast" ] = minContrast;

		j[ "fireRespawnTime" ] = fireRespawnTime.count();
		j[ "burnRepeatDelay" ] = burnRepeatDelay.count();
		j[ "stopTimeout" ] = stopTimeout.count();
	}
};

static bool handleFileDialog( const char* label, std::string& result )
{
	if ( ImGuiFileDialog::Instance()->Display( label ) ) {
		auto isOk = ImGuiFileDialog::Instance()->IsOk();
		if ( isOk ) {
			result = ImGuiFileDialog::Instance()->GetFilePathName();
		}

		// close
		ImGuiFileDialog::Instance()->Close();
		return isOk;
	}
	return false;
}

static std::string labelWithDirectory( const char* label )
{
	std::stringstream ss;
	ss << ICON_IGFD_FOLDER_OPEN << " " << label;
	return ss.str();
}
static std::string labelWithSave( const char* label )
{
	std::stringstream ss;
	ss << ICON_IGFD_SAVE << " " << label;
	return ss.str();
}

static bool hasEnding( const std::string& fullString, const std::string& ending )
{
	if ( fullString.length() >= ending.length() ) {
		return ( 0 == fullString.compare( fullString.length() - ending.length(), ending.length(), ending ) );
	} else {
		return false;
	}
}

template < typename Functor >
static bool folderSelectButton( const char* label, std::string& result, Functor f )
{
	auto l = labelWithDirectory( label );
	if ( ImGui::Button( l.c_str() ) ) {
		f();
		ImGuiFileDialog::Instance()->OpenModal(
			l, ICON_IGFD_FOLDER_OPEN " Choose a Directory", nullptr, result );
	}
	if ( handleFileDialog( l.c_str(), result ) ) {
		if ( !hasEnding( result, "/" ) ) {
			result += '/';
		}
		return true;
	}
	return false;
}

static bool folderSelectButton( const char* label, std::string& result )
{
	return folderSelectButton( label, result, []() {} );
}

static cv::Point3f worldPosition( cv::Point2d pt )
{
	// ToDo: Transform a 2D Pixel coordinate into the 3D World Coordinate of the same camera
	return {};
}

static cv::Mat grabImage()
{
	// ToDo: Return the latest RGB or RGBA image of the scene.
	// Should also set isImageNewThisFrame to true if it is a new frame
	return cv::Mat();
}

static ImVec2 imguiSpaceToImageSize( int width, int height )
{
	auto available = ImGui::GetContentRegionAvail();
	auto factorX = available.x / width;
	auto factorY = available.y / height;

	if ( factorX < factorY ) {
		return ImVec2( available.x, height * factorX );
	} else {
		return ImVec2( width * factorY, available.y );
	}
};

static void updateTeach()
{
	ImGui::Separator();

	static int currentRegion = 1;
	if ( ImGui::Button( ICON_IGFD_CHEVRON_UP " Next Region" ) ) {
		using namespace std::chrono_literals;
		if ( projectionClient->wait_for_service( 0s ) ) {
			auto lastProjectionRequest = std::make_shared< ServiceCall::Request >();
			auto& proj = lastProjectionRequest->projections.emplace_back();
			proj.worksheet = "Template";
			proj.projection = std::to_string( currentRegion );
			auto lastResponseFuture = projectionClient->async_send_request( lastProjectionRequest );

			currentRegion = currentRegion + 1;
			if ( currentRegion >= 4 ) {
				currentRegion = 1;
			}

			rclcpp::spin_until_future_complete( node, lastResponseFuture );
		}
	}

	if ( ImGui::Button( ICON_IGFD_CHEVRON_DOWN " Disable Region" ) ) {
		using namespace std::chrono_literals;
		if ( projectionClient->wait_for_service( 0s ) ) {
			auto lastProjectionRequest = std::make_shared< ServiceCall::Request >();
			auto& proj = lastProjectionRequest->projections.emplace_back();
			proj.worksheet = "Template";
			proj.projection = { "999" };
			auto lastResponseFuture = projectionClient->async_send_request( lastProjectionRequest );

			rclcpp::spin_until_future_complete( node, lastResponseFuture );
		}
	}

	ImGui::Separator();

	ImGui::DragScalar( "BB xMin", ImGuiDataType_Double, &GlobalSettings::tableRect.xMin, 0.01f );
	ImGui::DragScalar( "BB xMax", ImGuiDataType_Double, &GlobalSettings::tableRect.xMax, 0.01f );
	ImGui::DragScalar( "BB yMin", ImGuiDataType_Double, &GlobalSettings::tableRect.yMin, 0.01f );
	ImGui::DragScalar( "BB yMax", ImGuiDataType_Double, &GlobalSettings::tableRect.yMax, 0.01f );

	ImGui::Separator();

	if ( !lastImage.empty() ) {
		if ( learning ) {
			if ( !currentShapeModel ) {
				currentShapeModel = std::make_unique< ShapeModelDetector >( GlobalSettings::tableRect );
			}
			HalconImage img( lastImage );
			currentShapeModel->setImage( img );

			static int greenChannelThreshold = 190;
			if ( ImGui::Button( "Find detect region" ) ) {
				ShapeModelDetector::showImage( HalconImage( lastImage ) );
				currentShapeModel->findRegionROIFromImage( hMin, hMax, sMin, sMax, vMin, vMax );
			}

			ImGui::Text( "Color extraction" );
			ImGui::Indent();
			if ( ImGui::Button( "Display hue check" ) ) {
				ShapeModelDetector::showImage( HalconImage( lastImage ) );
				currentShapeModel->debugShowHueExtract(
					hMin, hMax, sMin, sMax, vMin, vMax, GlobalSettings::dilationSize );
			}
			ImGui::Unindent();

			if ( ImGui::Button( "Take snapshot" ) ) {
				ShapeModelDetector::showImage( HalconImage( lastImage ) );
				currentShapeModel->addAutoShapeModel( GlobalSettings::scoreThreshold,
													  hMin,
													  hMax,
													  sMin,
													  sMax,
													  vMin,
													  vMax,
													  GlobalSettings::dilationSize,
													  GlobalSettings::minContrast );
			}

			ImGui::SameLine();

			ImGui::Text( "Current: %zu", currentShapeModel->modelCount() );

			static std::string savePath;
			if ( folderSelectButton( "Save Template", savePath, [ & ]() { savePath = baseSavePath; } ) ) {
				currentShapeModel->serialize( savePath );
				learning = false;
				currentShapeModel = nullptr;
			}
			if ( ImGui::Button( ICON_IGFD_CANCEL " Cancel" ) ) {
				learning = false;
				currentShapeModel = nullptr;
			}
		} else {
			if ( ImGui::Button( "Start learning" ) ) {
				currentShapeModel = nullptr;
				learning = true;
			}

			static std::string loadPath;
			if ( folderSelectButton( "Load Template", loadPath, [ & ]() { loadPath = baseSavePath; } ) ) {
				currentShapeModel = std::make_unique< ShapeModelDetector >( GlobalSettings::tableRect );
				currentShapeModel->deserialize( loadPath );
			}

			// Have a model
			if ( currentShapeModel ) {
				static int maxMatches = 1;
				ImGui::DragInt( "Max Matches", &maxMatches, 0.1f, 1, 5 );
				// have new image
				if ( isImageNewThisFrame ) {
					HalconImage img( lastImage );
					ShapeModelDetector::showImage( img );

					currentShapeModel->setImage( img );
					auto results = currentShapeModel->findShapeModel(
						GlobalSettings::scoreThreshold, maxMatches, SearchMode::Table, true );
					for ( const auto& result : results ) {
						auto pos3d = worldPosition( result.pos );
						geometry_msgs::msg::PointStamped msg;
						msg.header.stamp = node->now();
						msg.header.frame_id = "rgb_camera_link";
						msg.point.x = pos3d.x;
						msg.point.y = pos3d.y;
						msg.point.z = pos3d.z;

						publisher->publish( msg );
					}
				}
			}
		}
	}
}

#define NLOHMANN_JSON_FROM_WITH_DEFAULT( v1 ) \
	nlohmann_json_t.v1 = nlohmann_json_j.value( #v1, nlohmann_json_default_obj.v1 );

#define NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT( Type, ... )                                    \
	friend void to_json( nlohmann::json& nlohmann_json_j, const Type& nlohmann_json_t )             \
	{                                                                                               \
		NLOHMANN_JSON_EXPAND( NLOHMANN_JSON_PASTE( NLOHMANN_JSON_TO, __VA_ARGS__ ) )                \
	}                                                                                               \
	friend void from_json( const nlohmann::json& nlohmann_json_j, Type& nlohmann_json_t )           \
	{                                                                                               \
		Type nlohmann_json_default_obj;                                                             \
		NLOHMANN_JSON_EXPAND( NLOHMANN_JSON_PASTE( NLOHMANN_JSON_FROM_WITH_DEFAULT, __VA_ARGS__ ) ) \
	}

struct Interactable
{
	bool flammable = false;
	bool dousing = false;
	bool ignites = false;
	bool isBurning = false;
	bool isFire = false;
	std::string loopingSound;
	float loopingSoundVolume = 1.f;
	std::string dousingSound;
	float dousingSoundVolume = 1.f;

	ShapeModelResult::BoundingCircle circle;
	std::chrono::steady_clock::time_point burnProtectionUntil;
	std::chrono::steady_clock::time_point disabledUntil;
	std::chrono::steady_clock::time_point lastSeen;
	cv::Point3f position3D;
	Sound loopS;
	Sound dousingS;

	void from_json( const nlohmann::json& j )
	{
		flammable = j.value( "flammable", flammable );
		dousing = j.value( "dousing", dousing );
		ignites = j.value( "ignites", ignites );
		isBurning = j.value( "isBurning", isBurning );
		isFire = j.value( "isFire", isFire );

		loopingSound = j.value( "loopingSound", loopingSound );
		loopingSoundVolume = j.value( "loopingSoundVolume", loopingSoundVolume );
		if ( !loopingSound.empty() ) {
			auto path = baseSavePath + loopingSound;
			loopS = soundPlayer.load( path.c_str() );
			loopS.setVolume( loopingSoundVolume );
			loopS.setLooping( true );
		}

		dousingSound = j.value( "dousingSound", dousingSound );
		dousingSoundVolume = j.value( "dousingSoundVolume", dousingSoundVolume );
		if ( !dousingSound.empty() ) {
			auto path = baseSavePath + dousingSound;
			dousingS = soundPlayer.load( path.c_str() );
			dousingS.setVolume( dousingSoundVolume );
		}
	}
	void to_json( nlohmann::json& j ) const
	{
		j[ "flammable" ] = flammable;
		j[ "dousing" ] = dousing;
		j[ "ignites" ] = ignites;
		j[ "isBurning" ] = isBurning;
		j[ "isFire" ] = isFire;
		j[ "loopingSound" ] = loopingSound;
		j[ "loopingSoundVolume" ] = loopingSoundVolume;
		j[ "dousingSound" ] = dousingSound;
		j[ "dousingSoundVolume" ] = dousingSoundVolume;
	}
};

struct Trackable
{
	std::vector< Interactable > interactables;
	std::string name;
	bool hueCheck = false;
	size_t maxCount = 1;

	std::shared_ptr< ShapeModelDetector > detector;

	friend void from_json( const nlohmann::json& j, Trackable& t )
	{
		Interactable i;
		i.from_json( j );
		t.name = j.value( "name", "" );
		t.hueCheck = j.value( "hueCheck", false );
		t.maxCount = j.value( "maxCount", 1 );
		t.interactables = std::vector< Interactable >{ t.maxCount, i };
	}
	friend void to_json( nlohmann::json& j, const Trackable& t )
	{
		t.interactables.front().to_json( j );

		j[ "name" ] = t.name;
		j[ "hueCheck" ] = t.hueCheck;
		j[ "maxCount" ] = t.maxCount;
	}
};

struct StaticProjection
{
	Interactable interactable;
	std::string worksheet;
	std::string name;

	friend void from_json( const nlohmann::json& j, StaticProjection& t )
	{
		t.interactable.from_json( j );
		t.name = j.value( "name", "" );
		t.worksheet = j.value( "worksheet", "" );
	}
	friend void to_json( nlohmann::json& j, const StaticProjection& t )
	{
		t.interactable.to_json( j );

		j[ "name" ] = t.name;
		j[ "worksheet" ] = t.worksheet;
	}
};

struct State
{
	std::vector< Trackable > trackables;
	std::vector< StaticProjection > staticProjections;
	std::vector< Interactable* > activeInteractables;
	std::chrono::steady_clock::time_point pauseIn;
	bool paused = false;
	bool projectionsPaused = false;
	NLOHMANN_DEFINE_TYPE_INTRUSIVE_WITH_DEFAULT( State, trackables, staticProjections );
};

static void handleRunServiceRequest()
{
	using namespace std::chrono_literals;
	static bool hasActiveFuture = false;
	static std::shared_future< ServiceCall::Response::SharedPtr > future;
	static std::chrono::steady_clock::time_point lastFutureStart;
	while ( rosServiceWorkerRunning ) {
		auto now = std::chrono::steady_clock().now();
		if ( hasActiveFuture ) {
			if ( now - lastFutureStart > std::chrono::milliseconds( 500 ) ) {
				currentRunProjectionRequest = nullptr;
				hasActiveFuture = false;
				continue;
			}

			auto result = rclcpp::spin_until_future_complete( node, future, 100ms );
			if ( result == rclcpp::FutureReturnCode::SUCCESS ) {
				std::scoped_lock lock( serviceMutex );
				lastRunProjectionRequest = currentRunProjectionRequest;
				lastRunProjectionResult = future.get();
				currentRunProjectionRequest = nullptr;
				hasActiveFuture = false;
			} else {
				continue;
			}
		}
		std::unique_lock lock( serviceMutex );

		// Handle Service Call that might be in progress
		if ( currentRunProjectionRequest ) {
			future = projectionClient->async_send_request( currentRunProjectionRequest );
			hasActiveFuture = true;
			lastFutureStart = now;
		}
		lock.unlock();

		if ( !hasActiveFuture ) {
			std::this_thread::sleep_for( 10ms );
		}
	}
}

static State state;

static void loadStateFromPath( std::string root )
{
	namespace fs = std::filesystem;
	// Load Config
	using namespace nlohmann;
	using namespace std::chrono_literals;
	auto configPath = fs::path( root ) / "state.json";
	if ( fs::is_regular_file( configPath ) ) {
		json config;
		std::ifstream i( configPath );
		i >> config;

		state = config.get< State >();
		state.pauseIn = std::chrono::steady_clock().now() + GlobalSettings::stopTimeout;
		const auto rootPath = fs::path( root );
		for ( auto& trackable : state.trackables ) {
			auto candidate = rootPath / trackable.name;
			trackable.detector = std::make_shared< ShapeModelDetector >();
			if ( !trackable.detector->deserialize( candidate.string() ) ) {
				printf(
					"Trackable with name '%s' has no corresponding folder in root: "
					"'%s'",
					trackable.name.c_str(),
					root.c_str() );
			}
		}
	}
}

static void updateRun()
{
	namespace fs = std::filesystem;
	using namespace std::chrono_literals;
	static std::string root = baseSavePath;
	struct TemplateFolder
	{
		fs::path folder;
		ShapeModelDetector detector;
	};

	if ( folderSelectButton( "Load", root ) ) {
		loadStateFromPath( root );
	}

	if ( isImageNewThisFrame ) {
		using namespace std::chrono_literals;

		HalconImage img( lastImage );
		if ( !lastImage.empty() ) {
			ShapeModelDetector::showImage( img );
		}

		// Update Tracking
		{
			std::for_each( std::execution::par,
						   state.trackables.begin(),
						   state.trackables.end(),
						   [ & ]( Trackable& trackable ) {
							   trackable.detector->setImage( img );
							   auto results
								   = trackable.detector->findShapeModel( GlobalSettings::scoreThreshold,
																		 trackable.maxCount,
																		 SearchMode::Table,
																		 trackable.hueCheck );
							   for ( size_t i = 0; i < results.size(); i++ ) {
								   const auto& result = results[ i ];
								   auto& interactable = trackable.interactables[ i ];
								   // Publish 3D Point
								   interactable.position3D = worldPosition( result.pos );
								   interactable.lastSeen = std::chrono::steady_clock::now();
								   interactable.circle = result.circle;
								   interactable.circle.radius *= 1.1;
							   }
						   } );
		}

		// Update Active Interactables
		state.activeInteractables.clear();
		auto now = std::chrono::steady_clock::now();
		for ( auto& trackable : state.trackables ) {
			for ( auto& interactable : trackable.interactables ) {
				if ( ( now - interactable.lastSeen ) < 250ms ) {
					state.activeInteractables.push_back( &interactable );
				}
			}
		}

		// Check this before the static projections since they are "always" active
		if ( state.activeInteractables.empty() ) {
			if ( now > state.pauseIn && !state.paused ) {
				state.paused = true;
				auto resetInteractable = [ now ]( Interactable& interactable ) {
					interactable.isBurning = false;
					interactable.disabledUntil = now;
					interactable.dousingS.stop();
					interactable.loopS.stop();
				};

				for ( auto& trackable : state.trackables ) {
					for ( auto& interactable : trackable.interactables ) {
						resetInteractable( interactable );
					}
				}
				for ( auto& statics : state.staticProjections ) {
					resetInteractable( statics.interactable );
				}
			}
		} else {
			state.pauseIn = now + GlobalSettings::stopTimeout;
			state.paused = false;
			state.projectionsPaused = false;
		}

		if ( state.paused ) {
			if ( !state.projectionsPaused && !currentRunProjectionRequest ) {
				// Disable projection
				std::scoped_lock lock( serviceMutex );
				auto projectionRequest = std::make_shared< ServiceCall::Request >();
				auto& proj = projectionRequest->projections.emplace_back();
				proj.worksheet = "Template";
				proj.projection = { "999" };
				currentRunProjectionRequest = projectionRequest;
				state.projectionsPaused = true;
			}
			return;
		}

		for ( auto& projection : state.staticProjections ) {
			if ( projection.interactable.disabledUntil <= now ) {
				projection.interactable.loopS.play();
				state.activeInteractables.push_back( &projection.interactable );
			} else {
				projection.interactable.loopS.stop();
			}
		}

		// Run through all interactables and do state changes
		for ( auto first : state.activeInteractables ) {
			for ( auto second : state.activeInteractables ) {
				if ( first->circle.intersects( second->circle ) ) {
					if ( first->isBurning && second->flammable && second->burnProtectionUntil <= now ) {
						second->isBurning = true;
					}
					if ( first->ignites && second->flammable && second->burnProtectionUntil <= now ) {
						second->isBurning = true;
					}
					if ( first->dousing && second->isBurning ) {
						second->burnProtectionUntil = now + 10s;
						second->isBurning = false;
						second->dousingS.play();
					}
					if ( first->dousing && second->isFire ) {
						second->disabledUntil = now + 20s;
						second->dousingS.play();
					}
				}
			}
		}

		auto projectionRequest = std::make_shared< ServiceCall::Request >();

		for ( const auto& trackable : state.trackables ) {
			for ( auto& interactable : trackable.interactables ) {
				if ( ( now - interactable.lastSeen ) >= 250ms ) {
					continue;
				}

				const auto& circle = interactable.circle;
				auto pos3dCircle = worldPosition( circle.pos );

				auto outerCirclePos = circle.pos + cv::Point2d( circle.radius, circle.radius );
				auto pos3dCircleRadius = worldPosition( outerCirclePos );

				auto p = pos3dCircle - pos3dCircleRadius;
				auto radius = sqrt( p.x * p.x + p.y * p.y + p.z * p.z );
				if ( radius > 0.15 ) {
					continue;
				}

				auto& projection = projectionRequest->circles.emplace_back();
				projection.radius_m = radius;
				projection.point.x = pos3dCircle.x;
				projection.point.y = pos3dCircle.y;
				projection.point.z = pos3dCircle.z;

				// Check state, and eventually draw texture on top of it
				if ( interactable.isBurning ) {
					auto& p = projectionRequest->projections.emplace_back();
					p.projection = "Burning";
					p.worksheet = "States";
					p.has_position = true;
					p.position = projection.point;
				}
			}
		}

		for ( const auto& staticProjection : state.staticProjections ) {
			if ( staticProjection.interactable.disabledUntil <= now ) {
				auto& projection = projectionRequest->projections.emplace_back();
				projection.worksheet = staticProjection.worksheet;
				projection.projection = staticProjection.name;
			}
		}

		std::unique_lock lock( serviceMutex );
		if ( !currentRunProjectionRequest ) {
			currentRunProjectionRequest = projectionRequest;
		}

		auto result = lastRunProjectionResult;
		auto lastProjectionRequest = lastRunProjectionRequest;
		lock.unlock();

		if ( lastProjectionRequest && result && result->success ) {
			for ( size_t i = 0; i < result->projection_bounding_box.size(); i++ ) {
				// Set Bounding boxes of static projections
				const auto& bb = result->projection_bounding_box[ i ];
				const auto& name = lastProjectionRequest->projections[ i ].projection;
				auto it
					= std::find_if( state.staticProjections.begin(),
									state.staticProjections.end(),
									[ &name ]( const StaticProjection& proj ) { return proj.name == name; } );

				if ( it != state.staticProjections.end() ) {
					auto tl = cv::Point2d( bb.x, bb.y );
					auto br = cv::Point2d( bb.x + bb.w, bb.y + bb.h );
					auto center = ( tl + br ) / 2.0;

					auto radius = std::min( bb.w / 2.0, bb.h / 2.0 );
					it->interactable.circle = { center, radius };
				}
			}
		}
	}
}

static void update()
{
	static bool updateOgl = false;
	isImageNewThisFrame = false;

	ImGui::Begin( "Main Menu" );
	{
		static std::string filePath;
		auto l = labelWithDirectory( "Load general config" );
		if ( ImGui::Button( l.c_str() ) ) {
			filePath = baseSavePath;
			ImGuiFileDialog::Instance()->SetExtentionInfos( ".json", ImVec4( 1, 1, 0, 0.9f ) );
			ImGuiFileDialog::Instance()->OpenModal(
				l, ICON_IGFD_FOLDER_OPEN "Choose a file", "Json file (*.json){.json}", filePath );
		}
		if ( handleFileDialog( l.c_str(), filePath ) ) {
			if ( !hasEnding( filePath, ".json" ) ) {
				filePath += ".json";
			}
			nlohmann::json config;
			std::ifstream i( filePath );
			i >> config;

			GlobalSettings::from_json( config );
		}
	}

	ImGui::SameLine();

	{
		static std::string filePath;
		auto l = labelWithSave( "Save general config" );
		if ( ImGui::Button( l.c_str() ) ) {
			filePath = baseSavePath;
			ImGuiFileDialog::Instance()->SetExtentionInfos( ".json", ImVec4( 1, 1, 0, 0.9f ) );
			ImGuiFileDialog::Instance()->OpenModal(
				l, ICON_IGFD_FOLDER_OPEN "Choose a file", "Json file (*.json){.json}", filePath );
		}

		if ( handleFileDialog( l.c_str(), filePath ) ) {
			if ( !hasEnding( filePath, ".json" ) ) {
				filePath += ".json";
			}
			nlohmann::json config;
			GlobalSettings::to_json( config );
			std::ofstream i( filePath );
			i << config.dump( 4 );
		}
	}

	ImGui::Text( "Application average %.3f ms/frame (%.1f FPS)",
				 1000.0f / ImGui::GetIO().Framerate,
				 ImGui::GetIO().Framerate );

	ImGui::Checkbox( "OpenGL Rendering", &updateOgl );
	folderSelectButton( "Set Base Save Path", baseSavePath );
	ImGui::SameLine();
	ImGui::TextUnformatted( baseSavePath.c_str() );
	if ( ImGui::IsItemHovered() ) {
		ImGui::BeginTooltip();
		ImGui::PushTextWrapPos( ImGui::GetFontSize() * 35.0f );
		ImGui::TextUnformatted( baseSavePath.c_str() );
		ImGui::PopTextWrapPos();
		ImGui::EndTooltip();
	}

	// Image Grabbing
	auto undistorted = grabImage();
	if ( !undistorted.empty() ) {
		isImageNewThisFrame = true;
		lastImage = undistorted;
		if ( updateOgl ) {
			if ( !lastImageTexture ) {
				lastImageTexture = std::make_unique< Texture >();
			}
			if ( !hsvTexture ) {
				hsvTexture = std::make_unique< Texture >();
			}
			lastImageTexture->update( lastImage );
		}
	}

	if ( !lastImage.empty() && updateOgl ) {
		if ( ImGui::Begin( "Image" ) ) {
			if ( lastImageTexture ) {
				ImGui::Image( (void*)(intptr_t)lastImageTexture->texture(),
							  imguiSpaceToImageSize( lastImage.cols, lastImage.rows ) );
			}
		}
		ImGui::End();

		if ( ImGui::Begin( "HSV Picker" ) ) {
			int h1, h2, h3, h4;
			int s1, s2, s3, s4;
			int v1, v2, v3, v4;

			{
				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "H Min", &hMin, 0.1f, 0, 255 );
				ImGui::SameLine();
				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "H Max", &hMax, 0.1f, 0, 255 );

				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "S Min", &sMin, 0.1f, 0, 255 );
				ImGui::SameLine();
				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "S Max", &sMax, 0.1f, 0, 255 );

				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "V Min", &vMin, 0.1f, 0, 255 );
				ImGui::SameLine();
				ImGui::PushItemWidth( ImGui::GetWindowWidth() * 0.3f );
				ImGui::DragInt( "V Max", &vMax, 0.1f, 0, 255 );

				if ( hMin < hMax ) {
					h1 = h3 = hMin;
					h2 = h4 = hMax;
				} else {
					h1 = 0;
					h2 = hMax;
					h3 = hMin;
					h4 = 255;
				}
				if ( sMin < sMax ) {
					s1 = s3 = sMin;
					s2 = s4 = sMax;
				} else {
					s1 = 0;
					s2 = sMax;
					s3 = sMin;
					s4 = 255;
				}
				if ( vMin < vMax ) {
					v1 = v3 = vMin;
					v2 = v4 = vMax;
				} else {
					v1 = 0;
					v2 = vMax;
					v3 = vMin;
					v4 = 255;
				}
			}

			cv::Mat hsv;
			cv::cvtColor( lastImage, hsv, cv::COLOR_BGR2HSV );
			auto clone = lastImage.clone();

			auto clamp = [ hsv ]( int hmin, int hmax, int smin, int smax, int vmin, int vmax ) {
				cv::Mat mask;
				cv::inRange( hsv,
							 cv::Scalar( hmin / 255.0 * 180.0, smin, vmin ),
							 cv::Scalar( hmax / 255.0 * 180.0, smax, vmax ),
							 mask );
				return mask;
			};

			auto hMask = clamp( h1, h2, 0, 255, 0, 255 );
			cv::bitwise_or( hMask, clamp( h3, h4, 0, 255, 0, 255 ), hMask );

			auto sMask = clamp( 0, 255, s1, s2, 0, 255 );
			cv::bitwise_or( sMask, clamp( 0, 255, s3, s4, 0, 255 ), sMask );

			auto vMask = clamp( 0, 255, 0, 255, v1, v2 );
			cv::bitwise_or( vMask, clamp( 0, 255, 0, 255, v3, v4 ), vMask );

			cv::Mat accumulateMask( lastImage.size(), CV_8UC1 );
			accumulateMask.setTo( 255 );
			cv::bitwise_and( accumulateMask, hMask, accumulateMask );
			cv::bitwise_and( accumulateMask, sMask, accumulateMask );
			cv::bitwise_and( accumulateMask, vMask, accumulateMask );

			cv::bitwise_not( accumulateMask, accumulateMask );
			clone.setTo( 0, accumulateMask );

			if ( ImGui::Button( "Reset Range" ) ) {
				hMin = 0;
				hMax = 255;
				sMin = 0;
				sMax = 255;
				vMin = 0;
				vMax = 255;
			}
			ImGui::SameLine();
			if ( ImGui::Button( "Green" ) ) {
				hMin = 32 / 180.0 * 255;
				hMax = 83 / 180.0 * 255;
				sMin = 30;
				sMax = 255;
				vMin = 136;
				vMax = 255;
			}
			ImGui::SameLine();
			if ( ImGui::Button( "Orange" ) ) {
				hMin = 1 / 180.0 * 255;
				hMax = 14 / 180.0 * 255;
				sMin = 134;
				sMax = 255;
				vMin = 178;
				vMax = 255;
			}
			ImGui::SameLine();
			if ( ImGui::Button( "Red" ) ) {
				hMin = 171 / 180.0 * 255;
				hMax = 13 / 180.0 * 255;
				sMin = 52;
				sMax = 255;
				vMin = 145;
				vMax = 255;
			}
			ImGui::SameLine();
			if ( ImGui::Button( "Flower Blue" ) ) {
				hMin = 93 / 180.0 * 255;
				hMax = 111 / 180.0 * 255;
				sMin = 71;
				sMax = 255;
				vMin = 111;
				vMax = 255;
			}
			ImGui::SameLine();
			if ( ImGui::Button( "House Blue" ) ) {
				hMin = 68 / 180.0 * 255;
				hMax = 132 / 180.0 * 255;
				sMin = 144;
				sMax = 255;
				vMin = 51;
				vMax = 255;
			}

			if ( hsvTexture ) {
				hsvTexture->update( clone );
				ImGui::Image( (void*)(intptr_t)hsvTexture->texture(),
							  imguiSpaceToImageSize( lastImage.cols, lastImage.rows ) );
			}
		}
		ImGui::End();
	}

	ImGui::Separator();
	if ( mode == Mode::TEACH && ImGui::Button( "Switch to run" ) ) {
		mode = Mode::RUN;
	}
	if ( mode == Mode::RUN && ImGui::Button( "Switch to teaching" ) ) {
		mode = Mode::TEACH;
	}

	ImGui::Separator();

	const double min = 0.4;
	const double max = 1.0;
	ImGui::DragScalar(
		"Min Score", ImGuiDataType_Double, &GlobalSettings::scoreThreshold, 0.001f, &min, &max );
	ImGui::DragInt( "Dilation Size", &GlobalSettings::dilationSize, 0.1f, 1, 21 );
	ImGui::DragInt( "Min Contrast", &GlobalSettings::minContrast, 0.1f, 1, 255 );

	ImGui::Separator();

	switch ( mode ) {
		case Mode::TEACH: {
			updateTeach();
			break;
		}
		case Mode::RUN: {
			updateRun();
			break;
		}
	}

	ImGui::End();
}

static void addExeFolderToPath( const char* path )
{
	namespace fs = std::filesystem;
	auto dir = fs::weakly_canonical( fs::path( path ) ).parent_path();
	size_t requiredSize;

	getenv_s( &requiredSize, nullptr, 0, "PATH" );
	std::string p;
	p.resize( requiredSize );
	getenv_s( &requiredSize, p.data(), requiredSize, "PATH" );

	auto s = dir.string() + ";" + p;
	_putenv_s( "PATH", s.c_str() );
}

int main( int argc, char* argv[] )
{
	addExeFolderToPath( argv[ 0 ] );

	rclcpp::init( 0, nullptr );
	if ( !rclcpp::ok() ) {
		return -1;
	}
	node = rclcpp::Node::make_shared( "halcon_tracking" );
	publisher = node->create_publisher< geometry_msgs::msg::PointStamped >( "poseYellow", 10 );

	projectionClient = node->create_client< ServiceCall >( "/change_projection" );

	glfwSetErrorCallback( glfw_error_callback );
	if ( !glfwInit() ) return 1;

	const char* glsl_version = "#version 130";
	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 0 );

	GLFWwindow* window = glfwCreateWindow( 1280, 720, "Debug UI Assembly Prototype", NULL, NULL );
	if ( window == NULL ) return 1;
	glfwMakeContextCurrent( window );
	glfwSwapInterval( 1 );	// Enable vsync

	bool err = glewInit() != GLEW_OK;
	if ( err ) {
		printf( "Failed to initialize OpenGL loader!\n" );
		return 1;
	}

	try {
		// Initialize();
	} catch ( ... ) {
		RCLCPP_ERROR( node->get_logger(), "Failed to initialize Halcon" );
		return 2;
	}

	// Setup Dear ImGui context
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO();

	// Setup Dear ImGui style
	ImGui::StyleColorsDark();

	// Setup Platform/Renderer backends
	ImGui_ImplGlfw_InitForOpenGL( window, true );
	ImGui_ImplOpenGL3_Init( glsl_version );

	// Custom Icons
	ImGui::GetIO().Fonts->AddFontDefault();
	static const ImWchar icons_ranges[] = { ICON_MIN_IGFD, ICON_MAX_IGFD, 0 };
	ImFontConfig icons_config;
	icons_config.MergeMode = true;
	icons_config.PixelSnapH = true;
	ImGui::GetIO().Fonts->AddFontFromMemoryCompressedBase85TTF(
		IGFD_compressed_data_base85, 15.0f, &icons_config, icons_ranges );

	ImVec4 clear_color = ImVec4( 0.45f, 0.55f, 0.60f, 1.00f );

	rosServiceWorkerRunning = true;
	rosServiceWorkerThread = std::thread( handleRunServiceRequest );

	// Try to load global config
	auto file = baseSavePath + "/config.json";
	if ( std::filesystem::is_regular_file( file ) ) {
		nlohmann::json config;
		std::ifstream i( file );
		i >> config;

		GlobalSettings::from_json( config );
	}

	loadStateFromPath( baseSavePath );

	while ( !glfwWindowShouldClose( window ) ) {
		glfwPollEvents();

		// Start the Dear ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		update();

		// Rendering
		ImGui::Render();
		int display_w, display_h;
		glfwGetFramebufferSize( window, &display_w, &display_h );
		glViewport( 0, 0, display_w, display_h );
		glClearColor( clear_color.x * clear_color.w,
					  clear_color.y * clear_color.w,
					  clear_color.z * clear_color.w,
					  clear_color.w );
		glClear( GL_COLOR_BUFFER_BIT );
		ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

		glfwSwapBuffers( window );

		rclcpp::spin_some( node );
	}

	rosServiceWorkerRunning = false;
	rosServiceWorkerThread.join();

	rclcpp::shutdown();

	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	glfwDestroyWindow( window );
	glfwTerminate();

	return 0;
}
