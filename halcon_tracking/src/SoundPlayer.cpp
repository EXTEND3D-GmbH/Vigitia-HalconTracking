#include "SoundPlayer.h"

#define MINIAUDIO_IMPLEMENTATION
#include "3rd/miniaudio.h"

#include <stdexcept>
#include <vector>
struct Sound::Impl
{
	Impl( const char* file, ma_engine* engine )
	{
		auto result = ma_sound_init_from_file( engine, file, 0, NULL, NULL, &sound );
		if ( result != MA_SUCCESS ) {
			throw std::runtime_error( "Could not load sound" );
		}
		ma_sound_set_fade_in_milliseconds( &sound, 0, 1, 300 );
		isValid = true;
	}
	~Impl()
	{
		if ( isValid ) {
			ma_sound_uninit( &sound );
		}
	}
	Impl( const Impl& ) = delete;
	Impl& operator=( const Impl& ) = delete;

	Impl( Impl&& rhs ) noexcept
	{
		sound = rhs.sound;
		isValid = rhs.isValid;
		rhs.isValid = false;
	};

	Impl& operator=( Impl&& rhs ) noexcept
	{
		sound = rhs.sound;
		isValid = rhs.isValid;
		rhs.isValid = false;
		return *this;
	};

	void play()
	{
		if ( ma_sound_is_playing( &sound ) ) {
			return;
		}
		ma_sound_start( &sound );
	}

	void stop()
	{
		ma_sound_stop( &sound );
	}

	void setLooping( bool looping )
	{
		ma_sound_set_looping( &sound, looping );
	}

	void setVolume( float volume )
	{
		ma_sound_set_volume( &sound, volume );
	}
	bool isValid = false;
	ma_sound sound;
};

struct SoundPlayer::Impl
{
	Impl()
	{
		ma_result result = ma_engine_init( NULL, &engine );
		if ( result != MA_SUCCESS ) {
			throw std::runtime_error( "Failed to initialize audio engine" );
		}
	}
	~Impl()
	{
		ma_engine_uninit( &engine );
	}

	std::shared_ptr< Sound::Impl > load( const char* file )
	{
		return std::make_shared< Sound::Impl >( file, &engine );
	}

	ma_engine engine;
};
Sound::Sound() = default;
Sound::~Sound() = default;

Sound::Sound( const Sound& ) = default;
Sound::Sound( Sound&& ) = default;
Sound& Sound::operator=( Sound&& ) = default;
Sound& Sound::operator=( const Sound& ) = default;

void Sound::play()
{
	if ( m_pimpl ) m_pimpl->play();
}
void Sound::stop()
{
	if ( m_pimpl ) m_pimpl->stop();
}
void Sound::setLooping( bool looping )
{
	if ( m_pimpl ) m_pimpl->setLooping( looping );
}
void Sound::setVolume( float volume )
{
	if ( m_pimpl ) m_pimpl->setVolume( volume );
}

SoundPlayer::SoundPlayer() : m_pimpl( std::make_unique< Impl >() )
{}

SoundPlayer::~SoundPlayer()
{}

Sound SoundPlayer::load( const char* file )
{
	Sound result;
	result.m_pimpl = m_pimpl->load( file );
	return result;
}