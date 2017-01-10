#ifndef LIBZSNDFILE_HPP
#define LIBZSNDFILE_HPP

#include <string>
#include <memory>
#include <sndfile.h>

namespace zsndfile
{
    enum class sound_file_mode : int
    {
        read_only = SFM_READ,
        write_only = SFM_WRITE,
        read_write = SFM_RDWR
    };
    enum class seek_mode : int
    {
        from_start = SEEK_SET,
        from_current = SEEK_CUR,
        from_end = SEEK_END
    };

    enum class sound_file_format : int
    {
      wav = SF_FORMAT_WAV,          /* Microsoft WAV format (little endian). */
      aiff = SF_FORMAT_AIFF,        /* Apple/SGI AIFF format (big endian). */
      au = SF_FORMAT_AU,            /* Sun/NeXT AU format (big endian). */
      raw = SF_FORMAT_RAW,          /* RAW PCM data. */
      paf =  SF_FORMAT_PAF,         /* Ensoniq PARIS file format. */
      svx = SF_FORMAT_SVX,          /* Amiga IFF / SVX8 / SV16 format. */
      nist = SF_FORMAT_NIST,        /* Sphere NIST format. */
      voc = SF_FORMAT_VOC,          /* VOC files. */
      ircam = SF_FORMAT_IRCAM,      /* Berkeley/IRCAM/CARL */
      w64 = SF_FORMAT_W64,          /* Sonic Foundry's 64 bit RIFF/WAV */
      mat4 = SF_FORMAT_MAT4,        /* Matlab (tm) V4.2 / GNU Octave 2.0 */
      mat5 = SF_FORMAT_MAT5,        /* Matlab (tm) V5.0 / GNU Octave 2.1 */
      pvf = SF_FORMAT_PVF,          /* Portable Voice Format */
      xi = SF_FORMAT_XI,            /* Fasttracker 2 Extended Instrument */
      htk = SF_FORMAT_HTK,          /* HMM Tool Kit format */
      sds = SF_FORMAT_SDS,          /* Midi Sample Dump Standard */
      avr = SF_FORMAT_AVR,          /* Audio Visual Research */
      wavex = SF_FORMAT_WAVEX,      /* MS WAVE with WAVEFORMATEX */
      sd2 = SF_FORMAT_SD2,          /* Sound Designer 2 */
      flac = SF_FORMAT_FLAC,        /* FLAC lossless file format */
      caf = SF_FORMAT_CAF,          /* Core Audio File format */
      wve = SF_FORMAT_WVE,          /* Psion WVE format */
      ogg = SF_FORMAT_OGG,          /* Xiph OGG container */
      mpc2k = SF_FORMAT_MPC2K,      /* Akai MPC 2000 sampler */
      rf64 = SF_FORMAT_RF64,        /* RF64 WAV file */

      /* Subtypes from here on. */

      pcm_s8 = SF_FORMAT_PCM_S8,            /* Signed 8 bit data */
      pcm_16 = SF_FORMAT_PCM_16 ,           /* Signed 16 bit data */
      pcm_24 = SF_FORMAT_PCM_24,            /* Signed 24 bit data */
      pcm_32 = SF_FORMAT_PCM_32,            /* Signed 32 bit data */

      pcm_u8 = SF_FORMAT_PCM_U8,            /* Unsigned 8 bit data (WAV and RAW only) */

      float_32 = SF_FORMAT_FLOAT,           /* 32 bit float data */
      float_64 = SF_FORMAT_DOUBLE,          /* 64 bit float data */

      ulaw = SF_FORMAT_ULAW,                /* U-Law encoded. */
      alaw = SF_FORMAT_ALAW,                /* A-Law encoded. */
      ima_adpcm = SF_FORMAT_IMA_ADPCM ,     /* IMA ADPCM. */
      ms_adpcm = SF_FORMAT_MS_ADPCM ,       /* Microsoft ADPCM. */

      gsm610 = SF_FORMAT_GSM610 ,           /* GSM 6.10 encoding. */
      vox_adpcm = SF_FORMAT_VOX_ADPCM ,     /* Oki Dialogic ADPCM encoding. */

      g721_32 = SF_FORMAT_G721_32 ,         /* 32kbs G721 ADPCM encoding. */
      g723_24 = SF_FORMAT_G723_24 ,         /* 24kbs G723 ADPCM encoding. */
      g723_40 = SF_FORMAT_G723_40 ,         /* 40kbs G723 ADPCM encoding. */

      dwvw_12 = SF_FORMAT_DWVW_12 ,         /* 12 bit Delta Width Variable Word encoding. */
      dwvw_16 = SF_FORMAT_DWVW_16 ,         /* 16 bit Delta Width Variable Word encoding. */
      dwvw_14 = SF_FORMAT_DWVW_24 ,         /* 24 bit Delta Width Variable Word encoding. */
      dwvw_n = SF_FORMAT_DWVW_N ,           /* N bit Delta Width Variable Word encoding. */

      dpcm_8 = SF_FORMAT_DPCM_8 ,           /* 8 bit differential PCM (XI only) */
      dpcm_16 = SF_FORMAT_DPCM_16 ,         /* 16 bit differential PCM (XI only) */

      vorbis = SF_FORMAT_VORBIS ,           /* Xiph Vorbis encoding. */

      /* Endian-ness options. */

      file_endian =  SF_ENDIAN_FILE ,       /* Default file endian-ness. */
      little_endian = SF_ENDIAN_LITTLE ,    /* Force little endian-ness. */
      big_endian = SF_ENDIAN_BIG ,          /* Force big endian-ness. */
      apu_endian = SF_ENDIAN_CPU ,          /* Force CPU endian-ness. */

      submask = SF_FORMAT_SUBMASK,
      typemask = SF_FORMAT_TYPEMASK,
      endmask = SF_FORMAT_ENDMASK
    };
    //enable or-ing of sound_file_format
    int operator | (const sound_file_format& lhs, const sound_file_format& rhs)
    {
        return static_cast<int>(lhs) | static_cast<int>(rhs);
    }


    namespace detail
    {
        template<typename sample_t>
        sf_count_t zsf_read(SNDFILE*,sample_t*,sf_count_t);
        template<>
        sf_count_t zsf_read<short>(SNDFILE* file, short* buff, sf_count_t count)
        {
            return sf_read_short(file,buff,count);
        }
        template<>
        sf_count_t zsf_read<int>(SNDFILE* file, int* buff, sf_count_t count)
        {
            return sf_read_int(file,buff,count);
        }
        template<>
        sf_count_t zsf_read<float>(SNDFILE* file, float* buff, sf_count_t count)
        {
            return sf_read_float(file,buff,count);
        }
        template<>
        sf_count_t zsf_read<double>(SNDFILE* file, double* buff, sf_count_t count)
        {
            return sf_read_double(file,buff,count);
        }

        template<typename sample_t>
        sf_count_t zsf_readf(SNDFILE*,sample_t*,sf_count_t);
        template<>
        sf_count_t zsf_readf<short>(SNDFILE* file, short* buff, sf_count_t count)
        {
            return sf_readf_short(file,buff,count);
        }
        template<>
        sf_count_t zsf_readf<int>(SNDFILE* file, int* buff, sf_count_t count)
        {
            return sf_readf_int(file,buff,count);
        }
        template<>
        sf_count_t zsf_readf<float>(SNDFILE* file, float* buff, sf_count_t count)
        {
            return sf_readf_float(file,buff,count);
        }
        template<>
        sf_count_t zsf_readf<double>(SNDFILE* file, double* buff, sf_count_t count)
        {
            return sf_readf_double(file,buff,count);
        }

        //write
        template<typename sample_t>
        sf_count_t zsf_write(SNDFILE*,sample_t*,sf_count_t);
        template<>
        sf_count_t zsf_write<short>(SNDFILE* file, short* buff, sf_count_t count)
        {
            return sf_write_short(file,buff,count);
        }
        template<>
        sf_count_t zsf_write<int>(SNDFILE* file, int* buff, sf_count_t count)
        {
            return sf_write_int(file,buff,count);
        }
        template<>
        sf_count_t zsf_write<float>(SNDFILE* file, float* buff, sf_count_t count)
        {
            return sf_write_float(file,buff,count);
        }
        template<>
        sf_count_t zsf_write<double>(SNDFILE* file, double* buff, sf_count_t count)
        {
            return sf_write_double(file,buff,count);
        }

        template<typename sample_t>
        sf_count_t zsf_writef(SNDFILE*,sample_t*,sf_count_t);
        template<>
        sf_count_t zsf_writef<short>(SNDFILE* file, short* buff, sf_count_t count)
        {
            return sf_writef_short(file,buff,count);
        }
        template<>
        sf_count_t zsf_writef<int>(SNDFILE* file, int* buff, sf_count_t count)
        {
            return sf_writef_int(file,buff,count);
        }
        template<>
        sf_count_t zsf_writef<float>(SNDFILE* file, float* buff, sf_count_t count)
        {
            return sf_writef_float(file,buff,count);
        }
        template<>
        sf_count_t zsf_writef<double>(SNDFILE* file, double* buff, sf_count_t count)
        {
            return sf_writef_double(file,buff,count);
        }
    }

    template<typename sample_t>
    constexpr bool is_valid_sample_type() noexcept
    {
        return false;
    }
    template<>
    constexpr bool is_valid_sample_type<short>() noexcept
    {
        return true;
    }
    template<>
    constexpr bool is_valid_sample_type<int>() noexcept
    {
        return true;
    }
    template<>
    constexpr bool is_valid_sample_type<float>() noexcept
    {
        return true;
    }
    template<>
    constexpr bool is_valid_sample_type<double>() noexcept
    {
        return true;
    }



    template<sound_file_mode mode>
    class sound_file_handle
    {
    public:
        constexpr static bool is_readable() noexcept
        {
            return (mode == sound_file_mode::read_only) || (mode == sound_file_mode::read_write);
        }
        constexpr static bool is_writeable() noexcept
        {
            return (mode == sound_file_mode::write_only) || (mode == sound_file_mode::read_write);
        }

        explicit sound_file_handle(const std::string& path) : _file(nullptr)
        {
            _file = sf_open(path.c_str(), static_cast<int>(mode),&_info);
            int err = sf_error(_file);
            throw_if_error(err);
        }
        explicit sound_file_handle(const std::wstring& path) : _file(nullptr)
        {
            _file = sf_whcar_open(path.c_str(), static_cast<int>(mode),&_info);
            int err = sf_error(_file);
            throw_if_error(err);
        }
        explicit sound_file_handle(const int& fd,const int& close_fd) : _file(nullptr)
        {
            _file = sf_open(fd, static_cast<int>(mode),&_info,close_fd);
            int err = sf_error(_file);
            throw_if_error(err);
        }
        ~sound_file_handle()
        {
            if(sf_close(_file) != 0)
            {
                throw std::runtime_error(sf_strerror(_file));
            }
        }

        //not going to wrap the enums for this one just yet,
        int command(int cmd,void* data,int datasize)
        {
            return sd_command(_file,cmd,data,datasize);
        }

        //seek
        sf_count_t seek(const sf_count_t& nframes, const seek_mode& smode) noexcept
        {
            return sf_seek(_file,nframes,static_cast<int>(smode));
        }

        //read
        template<typename sample_t>
        auto read(const sf_count_t& sample_count) -> typename std::enable_if<is_readable(),std::pair<std::unique_ptr<sample_t[]>,sf_count_t>>::type
        {
            using detail::zsf_read;
            std::unique_ptr<sample_t[]> outbuff{new sample_t[sample_count]};
            auto ret = zsf_read(_file,outbuff.get(),sample_count);
            return std::make_pair(std::move(outbuff),ret);
        }
        template<typename sample_t>
        auto read(const sf_count_t& sample_count,sample_t* buffer) -> typename std::enable_if<is_readable(),sf_count_t>::type
        {
            using detail::zsf_read;
            return zsf_read(_file,buffer,sample_count);

        }
        template<typename sample_t>
        auto read_frames(const sf_count_t& frame_count) -> typename std::enable_if<is_readable(),std::pair<std::unique_ptr<sample_t[]>,sf_count_t>>::type
        {
            using detail::zsf_readf;
            std::unique_ptr<sample_t[]> outbuff{new sample_t[frame_count * channel_count()]};
            auto ret = zsf_readf(_file,outbuff.get(),frame_count * channel_count());
            return std::make_pair(std::move(outbuff),ret);
        }
        template<typename sample_t>
        auto read_frames(const sf_count_t& frame_count, sample_t* buffer) -> typename std::enable_if<is_readable(),sf_count_t>::type
        {
            using detail::zsf_readf;
            return zsf_readf(_file,buffer,frame_count);
        }

        //write
        template<typename sample_t>
        auto write(sample_t* data,const sf_count_t& sample_count) -> typename std::enable_if<is_writeable(),sf_count_t>::type
        {
            using detail::zsf_write;
            return zsf_write(_file,data,sample_count);
        }
        template<typename sample_t>
        auto write_frames(sample_t* data,const sf_count_t& frame_count) -> typename std::enable_if<is_writeable(),sf_count_t>::type
        {
            using detail::zsf_writef;
            return zsf_writef(_file,data,frame_count);
        }



        //file format info
        auto frame_count() const noexcept -> typename std::enable_if<is_readable(),const sf_count_t&>::type
        {
            return _info.frames;
        }
        auto frame_count(const sf_count_t& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.frames = value;
        }

        auto channel_count() const noexcept -> typename std::enable_if<is_readable(), const int&>::type
        {
            return _info.channels;
        }
        auto channel_count(const int& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.channels = value;
        }

        auto sample_rate() const noexcept -> typename std::enable_if<is_readable(), const int&>::type
        {
            return _info.samplerate;
        }
        auto sample_rate(const int& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.samplerate = value;
        }

        auto format() const noexcept -> typename std::enable_if<is_readable(), const int&>::type
        {
            return _info.format;
        }
        auto format(const int& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.format = value;
        }

        auto sections() const noexcept -> typename std::enable_if<is_readable(), const int&>::type
        {
            return _info.sections;
        }
        auto sections(const int& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.sections = value;
        }

        auto seekable() const noexcept -> typename std::enable_if<is_readable(), const int&>::type
        {
            return _info.seekable;
        }
        auto seekable(const int& value) const noexcept -> typename std::enable_if<is_writeable(),void>::type
        {
            _info.seekable = value;
        }



        //metadata

        //title
        auto title() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_TITLE);
        }
        auto title(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_TITLE,value.c_str());
            throw_if_error(err);
        }

        //copyright
        auto copyright() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_COPYRIGHT);
        }
        auto copyright(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_COPYRIGHT,value.c_str());
            throw_if_error(err);
        }

        //software
        auto software() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_SOFTWARE);
        }
        auto software(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_SOFTWARE,value.c_str());
            throw_if_error(err);
        }

        //artist
        auto artist() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_ARTIST);
        }
        auto artist(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_ARTIST,value.c_str());
            throw_if_error(err);
        }

        //comment
        auto comment() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_COMMENT);
        }
        auto comment(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_COMMENT,value.c_str());
            throw_if_error(err);
        }

        //date
        auto date() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_DATE);
        }
        auto date(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_DATE,value.c_str());
            throw_if_error(err);
        }

        //album
        auto album() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_ALBUM);
        }
        auto album(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_ALBUM,value.c_str());
            throw_if_error(err);
        }

        //license
        auto license() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_LICENSE);
        }
        auto license(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_LICENSE,value.c_str());
            throw_if_error(err);
        }

        //track_number
        auto track_number() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_TRACKNUMBER);
        }
        auto track_number(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_TRACKNUMBER,value.c_str());
            throw_if_error(err);
        }

        //genre
        auto genre() noexcept -> typename std::enable_if<is_readable(),std::string>::type
        {
            return sf_get_string(_file,SF_STR_GENRE);
        }
        auto genre(const std::string& value) -> typename std::enable_if<is_writeable(),void>::type
        {
            auto err = sf_set_string(_file,SF_STR_GENRE,value.c_str());
            throw_if_error(err);
        }
    protected:
        SNDFILE* _file;
        SF_INFO _info;

        inline void throw_if_error(int err)
        {
            if(err != SF_ERR_NO_ERROR)
            {
                throw std::runtime_error(sf_error_number(err));
            }
        }
    };


    //read only sound file object, indexable
    template<typename sample_t,std::size_t default_buffer_size=256>
    class sound_file
    {
    public:
        static_assert(is_valid_sample_type<sample_t>(),"Sample type provided is invalid");

        explicit sound_file(const std::string& filename,
                            const bool& cache_file):_file(filename),
                                                    _is_file_cached(cache_file_if_requested(cache_file)),
                                                    _current_idx(0)
        {}
        explicit sound_file(const std::wstring& filename,
                            const bool& cache_file):_file(filename),
                                                    _is_file_cached(cache_file_if_requested(cache_file)),
                                                    _current_idx(0)
        {}
        explicit sound_file(const int& fd,
                            const int& close_fd
                            ,const bool& cache_file):_file(fd,close_fd),
                                                     _is_file_cached(cache_file_if_requested(cache_file)),
                                                     _current_idx(0)
        {}

        //will allow file to be indexed
        //if file is cached a simple lookup will happen
        //if file is not cached, a read will happen
        //NOTE: idea, maybe cache a section of the file if the whole file is not cached?
        //            this will allow for faster lookup?

        //read_only indexing
        const sample_t& operator[](const std::size_t& idx)
        {
            return lookup(idx);
        }
        sound_file_handle<sound_file_mode::read_only>& file() noexcept
        {
            return _file;
        }
        const bool& is_cached() const noexcept
        {
            return _is_file_cached;
        }
    protected:
        sound_file_handle<sound_file_mode::read_write> _file;
        std::unique_ptr<sample_t[]> _buffer;
        const bool _is_file_cached;
        std::size_t _current_idx;

        bool cache_file_if_requested(const bool& should_cache)
        {
            if(should_cache)
            {
                _buffer = std::unique_ptr<sample_t[]>{new(std::nothrow) sample_t[_file.channel_count() * _file.frame_count()]};
                if(_buffer != nullptr)
                {
                    _file.seek(0,seek_mode::from_start);
                    _file.read_frames<sample_t>(_file.frame_count(),_buffer.get());
                    return true;
                }
                else
                {
                    return false;
                }
            }
            else
            {
                return false;
            }
        }

        inline sample_t& lookup(const std::size_t& idx)
        {
            if(_is_file_cached)
            {
                return _buffer[idx];
            }
            else
            {
                //do we already have that section loaded
                if (idx >= _current_idx && idx < _current_idx + default_buffer_size)
                {
                    return _buffer[ idx - _current_idx];
                }
                else
                {
                    _file.seek(idx,seek_mode::from_start);
                    auto&& data = _file.read<sample_t>(default_buffer_size);
                    std::swap(_buffer,data);
                    return _buffer[0];
                }
            }
        }

    };

}

#endif
