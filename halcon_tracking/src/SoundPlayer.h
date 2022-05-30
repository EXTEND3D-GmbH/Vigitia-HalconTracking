#pragma once

#include <memory>

class Sound
{
	friend class SoundPlayer;

public:
	void play();
	void stop();
	void setLooping( bool looping );
	void setVolume( float volume );
	Sound();
	~Sound();
	Sound( Sound&& );
	Sound( const Sound& );
	Sound& operator=( Sound&& );
	Sound& operator=( const Sound& );

private:
	struct Impl;
	std::shared_ptr< Impl > m_pimpl;
};

class SoundPlayer
{
public:
	SoundPlayer();
	~SoundPlayer();

	Sound load( const char* file );

private:
	struct Impl;
	std::unique_ptr< Impl > m_pimpl;
};