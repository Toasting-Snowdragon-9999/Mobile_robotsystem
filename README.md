# Mobile_robotsystem
3. Semester projekt i mobile robotsystemer

The documentation can be found on the following URL: 
https://toasting-snowdragon-9999.github.io/Mobile_robotsystem/index.html


What is this repository for?

    SFML audio
    PortAudio
    libsndfile


How do I get set up?

    install SFML on linux using: "sudo apt-get install libsfml-dev"
    install libsndfile on linux using: "sudo apt install libsndfile-dev"
    if it doesnt work use: "sudo apt install autoconf autogen automake build-essential libasound2-dev \
                            libflac-dev libogg-dev libtool libvorbis-dev libopus-dev libmp3lame-dev \
                            libmpg123-dev pkg-config "
    install json dev for reading json file: "sudo apt-get install nlohmann-json3-dev"


    PortAudio: (might not work in WSL)
    
        install follow these steps: 
        "sudo apt update
        sudo apt install build-essential libasound2-dev libjack-jackd2-dev"
        "git clone https://github.com/PortAudio/portaudio.git
        cd portaudio"
        "./configure && make"       # Need make for this "sudo make install"
        "cd bin
        ./pa_devs
        ./pa_minlat"

        Finally write: "sudo apt install portaudio19-dev"
        


Contribution guidelines

    guidelines for git and code structure found in Docs.

Who do I talk to?

    Repo owner or admin
    Other community or team contact

/*



*/
