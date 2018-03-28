
/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_SIM_FRAME_HPP
#define OPENDLV_SIM_FRAME_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace sim {
using namespace std::string_literals; // NOLINT
class LIB_API Frame {
    public:
        Frame() = default;
        Frame(const Frame&) = default;
        Frame& operator=(const Frame&) = default;
        Frame(Frame&&) noexcept = default; // NOLINT
        Frame& operator=(Frame&&) noexcept = default; // NOLINT
        ~Frame() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Frame& x(const float &v) noexcept;
        float x() const noexcept;
        
        Frame& y(const float &v) noexcept;
        float y() const noexcept;
        
        Frame& z(const float &v) noexcept;
        float z() const noexcept;
        
        Frame& roll(const float &v) noexcept;
        float roll() const noexcept;
        
        Frame& pitch(const float &v) noexcept;
        float pitch() const noexcept;
        
        Frame& yaw(const float &v) noexcept;
        float yaw() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("x"s), m_x, visitor);
            
            doVisit(2, std::move("float"s), std::move("y"s), m_y, visitor);
            
            doVisit(3, std::move("float"s), std::move("z"s), m_z, visitor);
            
            doVisit(4, std::move("float"s), std::move("roll"s), m_roll, visitor);
            
            doVisit(5, std::move("float"s), std::move("pitch"s), m_pitch, visitor);
            
            doVisit(6, std::move("float"s), std::move("yaw"s), m_yaw, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("x"s), m_x, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("y"s), m_y, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("z"s), m_z, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("roll"s), m_roll, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("float"s), std::move("pitch"s), m_pitch, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("yaw"s), m_yaw, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_x{ 0.0f }; // field identifier = 1.
        
        float m_y{ 0.0f }; // field identifier = 2.
        
        float m_z{ 0.0f }; // field identifier = 3.
        
        float m_roll{ 0.0f }; // field identifier = 4.
        
        float m_pitch{ 0.0f }; // field identifier = 5.
        
        float m_yaw{ 0.0f }; // field identifier = 6.
        
};
}}

template<>
struct isVisitable<opendlv::sim::Frame> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::sim::Frame> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_SIM_KINEMATICSTATE_HPP
#define OPENDLV_SIM_KINEMATICSTATE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace sim {
using namespace std::string_literals; // NOLINT
class LIB_API KinematicState {
    public:
        KinematicState() = default;
        KinematicState(const KinematicState&) = default;
        KinematicState& operator=(const KinematicState&) = default;
        KinematicState(KinematicState&&) noexcept = default; // NOLINT
        KinematicState& operator=(KinematicState&&) noexcept = default; // NOLINT
        ~KinematicState() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        KinematicState& vx(const float &v) noexcept;
        float vx() const noexcept;
        
        KinematicState& vy(const float &v) noexcept;
        float vy() const noexcept;
        
        KinematicState& vz(const float &v) noexcept;
        float vz() const noexcept;
        
        KinematicState& rollRate(const float &v) noexcept;
        float rollRate() const noexcept;
        
        KinematicState& pitchRate(const float &v) noexcept;
        float pitchRate() const noexcept;
        
        KinematicState& yawRate(const float &v) noexcept;
        float yawRate() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("vx"s), m_vx, visitor);
            
            doVisit(2, std::move("float"s), std::move("vy"s), m_vy, visitor);
            
            doVisit(3, std::move("float"s), std::move("vz"s), m_vz, visitor);
            
            doVisit(4, std::move("float"s), std::move("rollRate"s), m_rollRate, visitor);
            
            doVisit(5, std::move("float"s), std::move("pitchRate"s), m_pitchRate, visitor);
            
            doVisit(6, std::move("float"s), std::move("yawRate"s), m_yawRate, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("vx"s), m_vx, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("vy"s), m_vy, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("vz"s), m_vz, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("rollRate"s), m_rollRate, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("float"s), std::move("pitchRate"s), m_pitchRate, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("yawRate"s), m_yawRate, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_vx{ 0.0f }; // field identifier = 1.
        
        float m_vy{ 0.0f }; // field identifier = 2.
        
        float m_vz{ 0.0f }; // field identifier = 3.
        
        float m_rollRate{ 0.0f }; // field identifier = 4.
        
        float m_pitchRate{ 0.0f }; // field identifier = 5.
        
        float m_yawRate{ 0.0f }; // field identifier = 6.
        
};
}}

template<>
struct isVisitable<opendlv::sim::KinematicState> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::sim::KinematicState> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_BODY_COMPONENTINFO_HPP
#define OPENDLV_BODY_COMPONENTINFO_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace body {
using namespace std::string_literals; // NOLINT
class LIB_API ComponentInfo {
    public:
        ComponentInfo() = default;
        ComponentInfo(const ComponentInfo&) = default;
        ComponentInfo& operator=(const ComponentInfo&) = default;
        ComponentInfo(ComponentInfo&&) noexcept = default; // NOLINT
        ComponentInfo& operator=(ComponentInfo&&) noexcept = default; // NOLINT
        ~ComponentInfo() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ComponentInfo& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        
        ComponentInfo& x(const float &v) noexcept;
        float x() const noexcept;
        
        ComponentInfo& y(const float &v) noexcept;
        float y() const noexcept;
        
        ComponentInfo& z(const float &v) noexcept;
        float z() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            doVisit(2, std::move("float"s), std::move("x"s), m_x, visitor);
            
            doVisit(3, std::move("float"s), std::move("y"s), m_y, visitor);
            
            doVisit(4, std::move("float"s), std::move("z"s), m_z, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("x"s), m_x, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("y"s), m_y, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("z"s), m_z, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_description{ ""s }; // field identifier = 1.
        
        float m_x{ 0.0f }; // field identifier = 2.
        
        float m_y{ 0.0f }; // field identifier = 3.
        
        float m_z{ 0.0f }; // field identifier = 4.
        
};
}}

template<>
struct isVisitable<opendlv::body::ComponentInfo> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::body::ComponentInfo> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_BODY_ACTUATORINFO_HPP
#define OPENDLV_BODY_ACTUATORINFO_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace body {
using namespace std::string_literals; // NOLINT
class LIB_API ActuatorInfo {
    public:
        ActuatorInfo() = default;
        ActuatorInfo(const ActuatorInfo&) = default;
        ActuatorInfo& operator=(const ActuatorInfo&) = default;
        ActuatorInfo(ActuatorInfo&&) noexcept = default; // NOLINT
        ActuatorInfo& operator=(ActuatorInfo&&) noexcept = default; // NOLINT
        ~ActuatorInfo() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ActuatorInfo& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        
        ActuatorInfo& x(const float &v) noexcept;
        float x() const noexcept;
        
        ActuatorInfo& y(const float &v) noexcept;
        float y() const noexcept;
        
        ActuatorInfo& z(const float &v) noexcept;
        float z() const noexcept;
        
        ActuatorInfo& signalId(const uint32_t &v) noexcept;
        uint32_t signalId() const noexcept;
        
        ActuatorInfo& minValue(const float &v) noexcept;
        float minValue() const noexcept;
        
        ActuatorInfo& maxValue(const float &v) noexcept;
        float maxValue() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            doVisit(2, std::move("float"s), std::move("x"s), m_x, visitor);
            
            doVisit(3, std::move("float"s), std::move("y"s), m_y, visitor);
            
            doVisit(4, std::move("float"s), std::move("z"s), m_z, visitor);
            
            doVisit(5, std::move("uint32_t"s), std::move("signalId"s), m_signalId, visitor);
            
            doVisit(6, std::move("float"s), std::move("minValue"s), m_minValue, visitor);
            
            doVisit(7, std::move("float"s), std::move("maxValue"s), m_maxValue, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("x"s), m_x, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("y"s), m_y, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("z"s), m_z, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("uint32_t"s), std::move("signalId"s), m_signalId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("minValue"s), m_minValue, preVisit, visit, postVisit);
            
            doTripletForwardVisit(7, std::move("float"s), std::move("maxValue"s), m_maxValue, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_description{ ""s }; // field identifier = 1.
        
        float m_x{ 0.0f }; // field identifier = 2.
        
        float m_y{ 0.0f }; // field identifier = 3.
        
        float m_z{ 0.0f }; // field identifier = 4.
        
        uint32_t m_signalId{ 0 }; // field identifier = 5.
        
        float m_minValue{ 0.0f }; // field identifier = 6.
        
        float m_maxValue{ 0.0f }; // field identifier = 7.
        
};
}}

template<>
struct isVisitable<opendlv::body::ActuatorInfo> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::body::ActuatorInfo> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_BODY_SENSORINFO_HPP
#define OPENDLV_BODY_SENSORINFO_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace body {
using namespace std::string_literals; // NOLINT
class LIB_API SensorInfo {
    public:
        SensorInfo() = default;
        SensorInfo(const SensorInfo&) = default;
        SensorInfo& operator=(const SensorInfo&) = default;
        SensorInfo(SensorInfo&&) noexcept = default; // NOLINT
        SensorInfo& operator=(SensorInfo&&) noexcept = default; // NOLINT
        ~SensorInfo() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SensorInfo& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        
        SensorInfo& x(const float &v) noexcept;
        float x() const noexcept;
        
        SensorInfo& y(const float &v) noexcept;
        float y() const noexcept;
        
        SensorInfo& z(const float &v) noexcept;
        float z() const noexcept;
        
        SensorInfo& signalId(const uint32_t &v) noexcept;
        uint32_t signalId() const noexcept;
        
        SensorInfo& accuracyStd(const float &v) noexcept;
        float accuracyStd() const noexcept;
        
        SensorInfo& minFrequency(const uint16_t &v) noexcept;
        uint16_t minFrequency() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            doVisit(2, std::move("float"s), std::move("x"s), m_x, visitor);
            
            doVisit(3, std::move("float"s), std::move("y"s), m_y, visitor);
            
            doVisit(4, std::move("float"s), std::move("z"s), m_z, visitor);
            
            doVisit(5, std::move("uint32_t"s), std::move("signalId"s), m_signalId, visitor);
            
            doVisit(6, std::move("float"s), std::move("accuracyStd"s), m_accuracyStd, visitor);
            
            doVisit(7, std::move("uint16_t"s), std::move("minFrequency"s), m_minFrequency, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("x"s), m_x, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("y"s), m_y, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("z"s), m_z, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("uint32_t"s), std::move("signalId"s), m_signalId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("accuracyStd"s), m_accuracyStd, preVisit, visit, postVisit);
            
            doTripletForwardVisit(7, std::move("uint16_t"s), std::move("minFrequency"s), m_minFrequency, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_description{ ""s }; // field identifier = 1.
        
        float m_x{ 0.0f }; // field identifier = 2.
        
        float m_y{ 0.0f }; // field identifier = 3.
        
        float m_z{ 0.0f }; // field identifier = 4.
        
        uint32_t m_signalId{ 0 }; // field identifier = 5.
        
        float m_accuracyStd{ 0.0f }; // field identifier = 6.
        
        uint16_t m_minFrequency{ 0 }; // field identifier = 7.
        
};
}}

template<>
struct isVisitable<opendlv::body::SensorInfo> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::body::SensorInfo> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_BODY_SIGNALINFO_HPP
#define OPENDLV_BODY_SIGNALINFO_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace body {
using namespace std::string_literals; // NOLINT
class LIB_API SignalInfo {
    public:
        SignalInfo() = default;
        SignalInfo(const SignalInfo&) = default;
        SignalInfo& operator=(const SignalInfo&) = default;
        SignalInfo(SignalInfo&&) noexcept = default; // NOLINT
        SignalInfo& operator=(SignalInfo&&) noexcept = default; // NOLINT
        ~SignalInfo() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SignalInfo& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        
        SignalInfo& signalId(const uint32_t &v) noexcept;
        uint32_t signalId() const noexcept;
        
        SignalInfo& accuracyStd(const float &v) noexcept;
        float accuracyStd() const noexcept;
        
        SignalInfo& minFrequency(const uint16_t &v) noexcept;
        uint16_t minFrequency() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            doVisit(2, std::move("uint32_t"s), std::move("signalId"s), m_signalId, visitor);
            
            doVisit(3, std::move("float"s), std::move("accuracyStd"s), m_accuracyStd, visitor);
            
            doVisit(4, std::move("uint16_t"s), std::move("minFrequency"s), m_minFrequency, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint32_t"s), std::move("signalId"s), m_signalId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("accuracyStd"s), m_accuracyStd, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("uint16_t"s), std::move("minFrequency"s), m_minFrequency, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_description{ ""s }; // field identifier = 1.
        
        uint32_t m_signalId{ 0 }; // field identifier = 2.
        
        float m_accuracyStd{ 0.0f }; // field identifier = 3.
        
        uint16_t m_minFrequency{ 0 }; // field identifier = 4.
        
};
}}

template<>
struct isVisitable<opendlv::body::SignalInfo> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::body::SignalInfo> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_ACCELERATIONREADING_HPP
#define OPENDLV_PROXY_ACCELERATIONREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API AccelerationReading {
    public:
        AccelerationReading() = default;
        AccelerationReading(const AccelerationReading&) = default;
        AccelerationReading& operator=(const AccelerationReading&) = default;
        AccelerationReading(AccelerationReading&&) noexcept = default; // NOLINT
        AccelerationReading& operator=(AccelerationReading&&) noexcept = default; // NOLINT
        ~AccelerationReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AccelerationReading& accelerationX(const float &v) noexcept;
        float accelerationX() const noexcept;
        
        AccelerationReading& accelerationY(const float &v) noexcept;
        float accelerationY() const noexcept;
        
        AccelerationReading& accelerationZ(const float &v) noexcept;
        float accelerationZ() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("accelerationX"s), m_accelerationX, visitor);
            
            doVisit(2, std::move("float"s), std::move("accelerationY"s), m_accelerationY, visitor);
            
            doVisit(3, std::move("float"s), std::move("accelerationZ"s), m_accelerationZ, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("accelerationX"s), m_accelerationX, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("accelerationY"s), m_accelerationY, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("accelerationZ"s), m_accelerationZ, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_accelerationX{ 0.0f }; // field identifier = 1.
        
        float m_accelerationY{ 0.0f }; // field identifier = 2.
        
        float m_accelerationZ{ 0.0f }; // field identifier = 3.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::AccelerationReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::AccelerationReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_ANGULARVELOCITYREADING_HPP
#define OPENDLV_PROXY_ANGULARVELOCITYREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API AngularVelocityReading {
    public:
        AngularVelocityReading() = default;
        AngularVelocityReading(const AngularVelocityReading&) = default;
        AngularVelocityReading& operator=(const AngularVelocityReading&) = default;
        AngularVelocityReading(AngularVelocityReading&&) noexcept = default; // NOLINT
        AngularVelocityReading& operator=(AngularVelocityReading&&) noexcept = default; // NOLINT
        ~AngularVelocityReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AngularVelocityReading& angularVelocityX(const float &v) noexcept;
        float angularVelocityX() const noexcept;
        
        AngularVelocityReading& angularVelocityY(const float &v) noexcept;
        float angularVelocityY() const noexcept;
        
        AngularVelocityReading& angularVelocityZ(const float &v) noexcept;
        float angularVelocityZ() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("angularVelocityX"s), m_angularVelocityX, visitor);
            
            doVisit(2, std::move("float"s), std::move("angularVelocityY"s), m_angularVelocityY, visitor);
            
            doVisit(3, std::move("float"s), std::move("angularVelocityZ"s), m_angularVelocityZ, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("angularVelocityX"s), m_angularVelocityX, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("angularVelocityY"s), m_angularVelocityY, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("angularVelocityZ"s), m_angularVelocityZ, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_angularVelocityX{ 0.0f }; // field identifier = 1.
        
        float m_angularVelocityY{ 0.0f }; // field identifier = 2.
        
        float m_angularVelocityZ{ 0.0f }; // field identifier = 3.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::AngularVelocityReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::AngularVelocityReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_MAGNETICFIELDREADING_HPP
#define OPENDLV_PROXY_MAGNETICFIELDREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API MagneticFieldReading {
    public:
        MagneticFieldReading() = default;
        MagneticFieldReading(const MagneticFieldReading&) = default;
        MagneticFieldReading& operator=(const MagneticFieldReading&) = default;
        MagneticFieldReading(MagneticFieldReading&&) noexcept = default; // NOLINT
        MagneticFieldReading& operator=(MagneticFieldReading&&) noexcept = default; // NOLINT
        ~MagneticFieldReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        MagneticFieldReading& magneticFieldX(const float &v) noexcept;
        float magneticFieldX() const noexcept;
        
        MagneticFieldReading& magneticFieldY(const float &v) noexcept;
        float magneticFieldY() const noexcept;
        
        MagneticFieldReading& magneticFieldZ(const float &v) noexcept;
        float magneticFieldZ() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("magneticFieldX"s), m_magneticFieldX, visitor);
            
            doVisit(2, std::move("float"s), std::move("magneticFieldY"s), m_magneticFieldY, visitor);
            
            doVisit(3, std::move("float"s), std::move("magneticFieldZ"s), m_magneticFieldZ, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("magneticFieldX"s), m_magneticFieldX, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("magneticFieldY"s), m_magneticFieldY, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("magneticFieldZ"s), m_magneticFieldZ, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_magneticFieldX{ 0.0f }; // field identifier = 1.
        
        float m_magneticFieldY{ 0.0f }; // field identifier = 2.
        
        float m_magneticFieldZ{ 0.0f }; // field identifier = 3.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::MagneticFieldReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::MagneticFieldReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_ALTITUDEREADING_HPP
#define OPENDLV_PROXY_ALTITUDEREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API AltitudeReading {
    public:
        AltitudeReading() = default;
        AltitudeReading(const AltitudeReading&) = default;
        AltitudeReading& operator=(const AltitudeReading&) = default;
        AltitudeReading(AltitudeReading&&) noexcept = default; // NOLINT
        AltitudeReading& operator=(AltitudeReading&&) noexcept = default; // NOLINT
        ~AltitudeReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AltitudeReading& altitude(const float &v) noexcept;
        float altitude() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("altitude"s), m_altitude, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("altitude"s), m_altitude, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_altitude{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::AltitudeReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::AltitudeReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_PRESSUREREADING_HPP
#define OPENDLV_PROXY_PRESSUREREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PressureReading {
    public:
        PressureReading() = default;
        PressureReading(const PressureReading&) = default;
        PressureReading& operator=(const PressureReading&) = default;
        PressureReading(PressureReading&&) noexcept = default; // NOLINT
        PressureReading& operator=(PressureReading&&) noexcept = default; // NOLINT
        ~PressureReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PressureReading& pressure(const float &v) noexcept;
        float pressure() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("pressure"s), m_pressure, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("pressure"s), m_pressure, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_pressure{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PressureReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PressureReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_TEMPERATUREREADING_HPP
#define OPENDLV_PROXY_TEMPERATUREREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API TemperatureReading {
    public:
        TemperatureReading() = default;
        TemperatureReading(const TemperatureReading&) = default;
        TemperatureReading& operator=(const TemperatureReading&) = default;
        TemperatureReading(TemperatureReading&&) noexcept = default; // NOLINT
        TemperatureReading& operator=(TemperatureReading&&) noexcept = default; // NOLINT
        ~TemperatureReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        TemperatureReading& temperature(const float &v) noexcept;
        float temperature() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("temperature"s), m_temperature, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("temperature"s), m_temperature, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_temperature{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::TemperatureReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::TemperatureReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_TORQUEREADING_HPP
#define OPENDLV_PROXY_TORQUEREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API TorqueReading {
    public:
        TorqueReading() = default;
        TorqueReading(const TorqueReading&) = default;
        TorqueReading& operator=(const TorqueReading&) = default;
        TorqueReading(TorqueReading&&) noexcept = default; // NOLINT
        TorqueReading& operator=(TorqueReading&&) noexcept = default; // NOLINT
        ~TorqueReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        TorqueReading& torque(const float &v) noexcept;
        float torque() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("torque"s), m_torque, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("torque"s), m_torque, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_torque{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::TorqueReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::TorqueReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_VOLTAGEREADING_HPP
#define OPENDLV_PROXY_VOLTAGEREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API VoltageReading {
    public:
        VoltageReading() = default;
        VoltageReading(const VoltageReading&) = default;
        VoltageReading& operator=(const VoltageReading&) = default;
        VoltageReading(VoltageReading&&) noexcept = default; // NOLINT
        VoltageReading& operator=(VoltageReading&&) noexcept = default; // NOLINT
        ~VoltageReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        VoltageReading& torque(const float &v) noexcept;
        float torque() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("torque"s), m_torque, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("torque"s), m_torque, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_torque{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::VoltageReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::VoltageReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_ANGLEREADING_HPP
#define OPENDLV_PROXY_ANGLEREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API AngleReading {
    public:
        AngleReading() = default;
        AngleReading(const AngleReading&) = default;
        AngleReading& operator=(const AngleReading&) = default;
        AngleReading(AngleReading&&) noexcept = default; // NOLINT
        AngleReading& operator=(AngleReading&&) noexcept = default; // NOLINT
        ~AngleReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AngleReading& angle(const float &v) noexcept;
        float angle() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("angle"s), m_angle, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("angle"s), m_angle, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_angle{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::AngleReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::AngleReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_SWITCHSTATEREADING_HPP
#define OPENDLV_PROXY_SWITCHSTATEREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API SwitchStateReading {
    public:
        SwitchStateReading() = default;
        SwitchStateReading(const SwitchStateReading&) = default;
        SwitchStateReading& operator=(const SwitchStateReading&) = default;
        SwitchStateReading(SwitchStateReading&&) noexcept = default; // NOLINT
        SwitchStateReading& operator=(SwitchStateReading&&) noexcept = default; // NOLINT
        ~SwitchStateReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SwitchStateReading& state(const int16_t &v) noexcept;
        int16_t state() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("int16_t"s), std::move("state"s), m_state, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("int16_t"s), std::move("state"s), m_state, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        int16_t m_state{ 0 }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::SwitchStateReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::SwitchStateReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_PEDALPOSITIONREADING_HPP
#define OPENDLV_PROXY_PEDALPOSITIONREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PedalPositionReading {
    public:
        PedalPositionReading() = default;
        PedalPositionReading(const PedalPositionReading&) = default;
        PedalPositionReading& operator=(const PedalPositionReading&) = default;
        PedalPositionReading(PedalPositionReading&&) noexcept = default; // NOLINT
        PedalPositionReading& operator=(PedalPositionReading&&) noexcept = default; // NOLINT
        ~PedalPositionReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PedalPositionReading& position(const float &v) noexcept;
        float position() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("position"s), m_position, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("position"s), m_position, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_position{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PedalPositionReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PedalPositionReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDSTEERINGREADING_HPP
#define OPENDLV_PROXY_GROUNDSTEERINGREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSteeringReading {
    public:
        GroundSteeringReading() = default;
        GroundSteeringReading(const GroundSteeringReading&) = default;
        GroundSteeringReading& operator=(const GroundSteeringReading&) = default;
        GroundSteeringReading(GroundSteeringReading&&) noexcept = default; // NOLINT
        GroundSteeringReading& operator=(GroundSteeringReading&&) noexcept = default; // NOLINT
        ~GroundSteeringReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSteeringReading& groundSteering(const float &v) noexcept;
        float groundSteering() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundSteering"s), m_groundSteering, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundSteering"s), m_groundSteering, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundSteering{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundSteeringReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundSteeringReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDSPEEDREADING_HPP
#define OPENDLV_PROXY_GROUNDSPEEDREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSpeedReading {
    public:
        GroundSpeedReading() = default;
        GroundSpeedReading(const GroundSpeedReading&) = default;
        GroundSpeedReading& operator=(const GroundSpeedReading&) = default;
        GroundSpeedReading(GroundSpeedReading&&) noexcept = default; // NOLINT
        GroundSpeedReading& operator=(GroundSpeedReading&&) noexcept = default; // NOLINT
        ~GroundSpeedReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSpeedReading& groundSpeed(const float &v) noexcept;
        float groundSpeed() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundSpeed"s), m_groundSpeed, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundSpeed"s), m_groundSpeed, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundSpeed{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundSpeedReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundSpeedReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_WEIGHTREADING_HPP
#define OPENDLV_PROXY_WEIGHTREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API WeightReading {
    public:
        WeightReading() = default;
        WeightReading(const WeightReading&) = default;
        WeightReading& operator=(const WeightReading&) = default;
        WeightReading(WeightReading&&) noexcept = default; // NOLINT
        WeightReading& operator=(WeightReading&&) noexcept = default; // NOLINT
        ~WeightReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        WeightReading& torque(const float &v) noexcept;
        float torque() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("torque"s), m_torque, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("torque"s), m_torque, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_torque{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::WeightReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::WeightReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GEODETICHEADINGREADING_HPP
#define OPENDLV_PROXY_GEODETICHEADINGREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GeodeticHeadingReading {
    public:
        GeodeticHeadingReading() = default;
        GeodeticHeadingReading(const GeodeticHeadingReading&) = default;
        GeodeticHeadingReading& operator=(const GeodeticHeadingReading&) = default;
        GeodeticHeadingReading(GeodeticHeadingReading&&) noexcept = default; // NOLINT
        GeodeticHeadingReading& operator=(GeodeticHeadingReading&&) noexcept = default; // NOLINT
        ~GeodeticHeadingReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GeodeticHeadingReading& northHeading(const float &v) noexcept;
        float northHeading() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("northHeading"s), m_northHeading, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("northHeading"s), m_northHeading, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_northHeading{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GeodeticHeadingReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GeodeticHeadingReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GEODETICWGS84READING_HPP
#define OPENDLV_PROXY_GEODETICWGS84READING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GeodeticWgs84Reading {
    public:
        GeodeticWgs84Reading() = default;
        GeodeticWgs84Reading(const GeodeticWgs84Reading&) = default;
        GeodeticWgs84Reading& operator=(const GeodeticWgs84Reading&) = default;
        GeodeticWgs84Reading(GeodeticWgs84Reading&&) noexcept = default; // NOLINT
        GeodeticWgs84Reading& operator=(GeodeticWgs84Reading&&) noexcept = default; // NOLINT
        ~GeodeticWgs84Reading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GeodeticWgs84Reading& latitude(const double &v) noexcept;
        double latitude() const noexcept;
        
        GeodeticWgs84Reading& longitude(const double &v) noexcept;
        double longitude() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("double"s), std::move("latitude"s), m_latitude, visitor);
            
            doVisit(3, std::move("double"s), std::move("longitude"s), m_longitude, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("double"s), std::move("latitude"s), m_latitude, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("double"s), std::move("longitude"s), m_longitude, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        double m_latitude{ 0.0 }; // field identifier = 1.
        
        double m_longitude{ 0.0 }; // field identifier = 3.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GeodeticWgs84Reading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GeodeticWgs84Reading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_IMAGEREADINGSHARED_HPP
#define OPENDLV_PROXY_IMAGEREADINGSHARED_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API ImageReadingShared {
    public:
        ImageReadingShared() = default;
        ImageReadingShared(const ImageReadingShared&) = default;
        ImageReadingShared& operator=(const ImageReadingShared&) = default;
        ImageReadingShared(ImageReadingShared&&) noexcept = default; // NOLINT
        ImageReadingShared& operator=(ImageReadingShared&&) noexcept = default; // NOLINT
        ~ImageReadingShared() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ImageReadingShared& name(const std::string &v) noexcept;
        std::string name() const noexcept;
        
        ImageReadingShared& size(const uint32_t &v) noexcept;
        uint32_t size() const noexcept;
        
        ImageReadingShared& width(const uint32_t &v) noexcept;
        uint32_t width() const noexcept;
        
        ImageReadingShared& height(const uint32_t &v) noexcept;
        uint32_t height() const noexcept;
        
        ImageReadingShared& bytesPerPixel(const uint32_t &v) noexcept;
        uint32_t bytesPerPixel() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("name"s), m_name, visitor);
            
            doVisit(2, std::move("uint32_t"s), std::move("size"s), m_size, visitor);
            
            doVisit(3, std::move("uint32_t"s), std::move("width"s), m_width, visitor);
            
            doVisit(4, std::move("uint32_t"s), std::move("height"s), m_height, visitor);
            
            doVisit(5, std::move("uint32_t"s), std::move("bytesPerPixel"s), m_bytesPerPixel, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("name"s), m_name, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint32_t"s), std::move("size"s), m_size, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("uint32_t"s), std::move("width"s), m_width, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("uint32_t"s), std::move("height"s), m_height, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("uint32_t"s), std::move("bytesPerPixel"s), m_bytesPerPixel, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_name{ ""s }; // field identifier = 1.
        
        uint32_t m_size{ 0 }; // field identifier = 2.
        
        uint32_t m_width{ 0 }; // field identifier = 3.
        
        uint32_t m_height{ 0 }; // field identifier = 4.
        
        uint32_t m_bytesPerPixel{ 0 }; // field identifier = 5.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::ImageReadingShared> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::ImageReadingShared> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_POINTCLOUDREADING_HPP
#define OPENDLV_PROXY_POINTCLOUDREADING_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PointCloudReading {
    public:
        PointCloudReading() = default;
        PointCloudReading(const PointCloudReading&) = default;
        PointCloudReading& operator=(const PointCloudReading&) = default;
        PointCloudReading(PointCloudReading&&) noexcept = default; // NOLINT
        PointCloudReading& operator=(PointCloudReading&&) noexcept = default; // NOLINT
        ~PointCloudReading() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PointCloudReading& startAzimuth(const float &v) noexcept;
        float startAzimuth() const noexcept;
        
        PointCloudReading& endAzimuth(const float &v) noexcept;
        float endAzimuth() const noexcept;
        
        PointCloudReading& entriesPerAzimuth(const uint8_t &v) noexcept;
        uint8_t entriesPerAzimuth() const noexcept;
        
        PointCloudReading& distances(const std::string &v) noexcept;
        std::string distances() const noexcept;
        
        PointCloudReading& numberOfBitsForIntensity(const uint8_t &v) noexcept;
        uint8_t numberOfBitsForIntensity() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("startAzimuth"s), m_startAzimuth, visitor);
            
            doVisit(2, std::move("float"s), std::move("endAzimuth"s), m_endAzimuth, visitor);
            
            doVisit(3, std::move("uint8_t"s), std::move("entriesPerAzimuth"s), m_entriesPerAzimuth, visitor);
            
            doVisit(4, std::move("std::string"s), std::move("distances"s), m_distances, visitor);
            
            doVisit(5, std::move("uint8_t"s), std::move("numberOfBitsForIntensity"s), m_numberOfBitsForIntensity, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("startAzimuth"s), m_startAzimuth, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("endAzimuth"s), m_endAzimuth, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("uint8_t"s), std::move("entriesPerAzimuth"s), m_entriesPerAzimuth, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("std::string"s), std::move("distances"s), m_distances, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("uint8_t"s), std::move("numberOfBitsForIntensity"s), m_numberOfBitsForIntensity, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_startAzimuth{ 0.0f }; // field identifier = 1.
        
        float m_endAzimuth{ 0.0f }; // field identifier = 2.
        
        uint8_t m_entriesPerAzimuth{ 0 }; // field identifier = 3.
        
        std::string m_distances{ ""s }; // field identifier = 4.
        
        uint8_t m_numberOfBitsForIntensity{ 0 }; // field identifier = 5.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PointCloudReading> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PointCloudReading> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_POINTCLOUDREADINGSHARED_HPP
#define OPENDLV_PROXY_POINTCLOUDREADINGSHARED_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PointCloudReadingShared {
    public:
        PointCloudReadingShared() = default;
        PointCloudReadingShared(const PointCloudReadingShared&) = default;
        PointCloudReadingShared& operator=(const PointCloudReadingShared&) = default;
        PointCloudReadingShared(PointCloudReadingShared&&) noexcept = default; // NOLINT
        PointCloudReadingShared& operator=(PointCloudReadingShared&&) noexcept = default; // NOLINT
        ~PointCloudReadingShared() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PointCloudReadingShared& name(const std::string &v) noexcept;
        std::string name() const noexcept;
        
        PointCloudReadingShared& size(const uint32_t &v) noexcept;
        uint32_t size() const noexcept;
        
        PointCloudReadingShared& width(const uint32_t &v) noexcept;
        uint32_t width() const noexcept;
        
        PointCloudReadingShared& height(const uint32_t &v) noexcept;
        uint32_t height() const noexcept;
        
        PointCloudReadingShared& numberOfComponentsPerPoint(const uint8_t &v) noexcept;
        uint8_t numberOfComponentsPerPoint() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("std::string"s), std::move("name"s), m_name, visitor);
            
            doVisit(2, std::move("uint32_t"s), std::move("size"s), m_size, visitor);
            
            doVisit(3, std::move("uint32_t"s), std::move("width"s), m_width, visitor);
            
            doVisit(4, std::move("uint32_t"s), std::move("height"s), m_height, visitor);
            
            doVisit(5, std::move("uint8_t"s), std::move("numberOfComponentsPerPoint"s), m_numberOfComponentsPerPoint, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("std::string"s), std::move("name"s), m_name, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint32_t"s), std::move("size"s), m_size, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("uint32_t"s), std::move("width"s), m_width, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("uint32_t"s), std::move("height"s), m_height, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("uint8_t"s), std::move("numberOfComponentsPerPoint"s), m_numberOfComponentsPerPoint, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        std::string m_name{ ""s }; // field identifier = 1.
        
        uint32_t m_size{ 0 }; // field identifier = 2.
        
        uint32_t m_width{ 0 }; // field identifier = 3.
        
        uint32_t m_height{ 0 }; // field identifier = 4.
        
        uint8_t m_numberOfComponentsPerPoint{ 0 }; // field identifier = 5.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PointCloudReadingShared> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PointCloudReadingShared> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_PRESSUREREQUEST_HPP
#define OPENDLV_PROXY_PRESSUREREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PressureRequest {
    public:
        PressureRequest() = default;
        PressureRequest(const PressureRequest&) = default;
        PressureRequest& operator=(const PressureRequest&) = default;
        PressureRequest(PressureRequest&&) noexcept = default; // NOLINT
        PressureRequest& operator=(PressureRequest&&) noexcept = default; // NOLINT
        ~PressureRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PressureRequest& pressure(const float &v) noexcept;
        float pressure() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("pressure"s), m_pressure, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("pressure"s), m_pressure, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_pressure{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PressureRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PressureRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_TEMPERATUREREQUEST_HPP
#define OPENDLV_PROXY_TEMPERATUREREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API TemperatureRequest {
    public:
        TemperatureRequest() = default;
        TemperatureRequest(const TemperatureRequest&) = default;
        TemperatureRequest& operator=(const TemperatureRequest&) = default;
        TemperatureRequest(TemperatureRequest&&) noexcept = default; // NOLINT
        TemperatureRequest& operator=(TemperatureRequest&&) noexcept = default; // NOLINT
        ~TemperatureRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        TemperatureRequest& temperature(const float &v) noexcept;
        float temperature() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("temperature"s), m_temperature, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("temperature"s), m_temperature, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_temperature{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::TemperatureRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::TemperatureRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_TORQUEREQUEST_HPP
#define OPENDLV_PROXY_TORQUEREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API TorqueRequest {
    public:
        TorqueRequest() = default;
        TorqueRequest(const TorqueRequest&) = default;
        TorqueRequest& operator=(const TorqueRequest&) = default;
        TorqueRequest(TorqueRequest&&) noexcept = default; // NOLINT
        TorqueRequest& operator=(TorqueRequest&&) noexcept = default; // NOLINT
        ~TorqueRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        TorqueRequest& torque(const float &v) noexcept;
        float torque() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("torque"s), m_torque, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("torque"s), m_torque, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_torque{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::TorqueRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::TorqueRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_VOLTAGEREQUEST_HPP
#define OPENDLV_PROXY_VOLTAGEREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API VoltageRequest {
    public:
        VoltageRequest() = default;
        VoltageRequest(const VoltageRequest&) = default;
        VoltageRequest& operator=(const VoltageRequest&) = default;
        VoltageRequest(VoltageRequest&&) noexcept = default; // NOLINT
        VoltageRequest& operator=(VoltageRequest&&) noexcept = default; // NOLINT
        ~VoltageRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        VoltageRequest& torque(const float &v) noexcept;
        float torque() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("torque"s), m_torque, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("torque"s), m_torque, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_torque{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::VoltageRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::VoltageRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_ANGLEREQUEST_HPP
#define OPENDLV_PROXY_ANGLEREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API AngleRequest {
    public:
        AngleRequest() = default;
        AngleRequest(const AngleRequest&) = default;
        AngleRequest& operator=(const AngleRequest&) = default;
        AngleRequest(AngleRequest&&) noexcept = default; // NOLINT
        AngleRequest& operator=(AngleRequest&&) noexcept = default; // NOLINT
        ~AngleRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AngleRequest& angle(const float &v) noexcept;
        float angle() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("angle"s), m_angle, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("angle"s), m_angle, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_angle{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::AngleRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::AngleRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_SWITCHSTATEREQUEST_HPP
#define OPENDLV_PROXY_SWITCHSTATEREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API SwitchStateRequest {
    public:
        SwitchStateRequest() = default;
        SwitchStateRequest(const SwitchStateRequest&) = default;
        SwitchStateRequest& operator=(const SwitchStateRequest&) = default;
        SwitchStateRequest(SwitchStateRequest&&) noexcept = default; // NOLINT
        SwitchStateRequest& operator=(SwitchStateRequest&&) noexcept = default; // NOLINT
        ~SwitchStateRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SwitchStateRequest& state(const int16_t &v) noexcept;
        int16_t state() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("int16_t"s), std::move("state"s), m_state, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("int16_t"s), std::move("state"s), m_state, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        int16_t m_state{ 0 }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::SwitchStateRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::SwitchStateRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_PEDALPOSITIONREQUEST_HPP
#define OPENDLV_PROXY_PEDALPOSITIONREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PedalPositionRequest {
    public:
        PedalPositionRequest() = default;
        PedalPositionRequest(const PedalPositionRequest&) = default;
        PedalPositionRequest& operator=(const PedalPositionRequest&) = default;
        PedalPositionRequest(PedalPositionRequest&&) noexcept = default; // NOLINT
        PedalPositionRequest& operator=(PedalPositionRequest&&) noexcept = default; // NOLINT
        ~PedalPositionRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PedalPositionRequest& position(const float &v) noexcept;
        float position() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("position"s), m_position, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("position"s), m_position, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_position{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PedalPositionRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PedalPositionRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_PULSEWIDTHMODULATIONREQUEST_HPP
#define OPENDLV_PROXY_PULSEWIDTHMODULATIONREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API PulseWidthModulationRequest {
    public:
        PulseWidthModulationRequest() = default;
        PulseWidthModulationRequest(const PulseWidthModulationRequest&) = default;
        PulseWidthModulationRequest& operator=(const PulseWidthModulationRequest&) = default;
        PulseWidthModulationRequest(PulseWidthModulationRequest&&) noexcept = default; // NOLINT
        PulseWidthModulationRequest& operator=(PulseWidthModulationRequest&&) noexcept = default; // NOLINT
        ~PulseWidthModulationRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PulseWidthModulationRequest& dutyCycleNs(const uint32_t &v) noexcept;
        uint32_t dutyCycleNs() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("dutyCycleNs"s), m_dutyCycleNs, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("dutyCycleNs"s), m_dutyCycleNs, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_dutyCycleNs{ 0 }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::PulseWidthModulationRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::PulseWidthModulationRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDSTEERINGREQUEST_HPP
#define OPENDLV_PROXY_GROUNDSTEERINGREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSteeringRequest {
    public:
        GroundSteeringRequest() = default;
        GroundSteeringRequest(const GroundSteeringRequest&) = default;
        GroundSteeringRequest& operator=(const GroundSteeringRequest&) = default;
        GroundSteeringRequest(GroundSteeringRequest&&) noexcept = default; // NOLINT
        GroundSteeringRequest& operator=(GroundSteeringRequest&&) noexcept = default; // NOLINT
        ~GroundSteeringRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSteeringRequest& groundSteering(const float &v) noexcept;
        float groundSteering() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundSteering"s), m_groundSteering, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundSteering"s), m_groundSteering, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundSteering{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundSteeringRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundSteeringRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDSPEEDREQUEST_HPP
#define OPENDLV_PROXY_GROUNDSPEEDREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSpeedRequest {
    public:
        GroundSpeedRequest() = default;
        GroundSpeedRequest(const GroundSpeedRequest&) = default;
        GroundSpeedRequest& operator=(const GroundSpeedRequest&) = default;
        GroundSpeedRequest(GroundSpeedRequest&&) noexcept = default; // NOLINT
        GroundSpeedRequest& operator=(GroundSpeedRequest&&) noexcept = default; // NOLINT
        ~GroundSpeedRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSpeedRequest& groundSpeed(const float &v) noexcept;
        float groundSpeed() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundSpeed"s), m_groundSpeed, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundSpeed"s), m_groundSpeed, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundSpeed{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundSpeedRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundSpeedRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDACCELERATIONREQUEST_HPP
#define OPENDLV_PROXY_GROUNDACCELERATIONREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundAccelerationRequest {
    public:
        GroundAccelerationRequest() = default;
        GroundAccelerationRequest(const GroundAccelerationRequest&) = default;
        GroundAccelerationRequest& operator=(const GroundAccelerationRequest&) = default;
        GroundAccelerationRequest(GroundAccelerationRequest&&) noexcept = default; // NOLINT
        GroundAccelerationRequest& operator=(GroundAccelerationRequest&&) noexcept = default; // NOLINT
        ~GroundAccelerationRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundAccelerationRequest& groundAcceleration(const float &v) noexcept;
        float groundAcceleration() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundAcceleration"s), m_groundAcceleration, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundAcceleration"s), m_groundAcceleration, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundAcceleration{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundAccelerationRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundAccelerationRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_PROXY_GROUNDDECELERATIONREQUEST_HPP
#define OPENDLV_PROXY_GROUNDDECELERATIONREQUEST_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace proxy {
using namespace std::string_literals; // NOLINT
class LIB_API GroundDecelerationRequest {
    public:
        GroundDecelerationRequest() = default;
        GroundDecelerationRequest(const GroundDecelerationRequest&) = default;
        GroundDecelerationRequest& operator=(const GroundDecelerationRequest&) = default;
        GroundDecelerationRequest(GroundDecelerationRequest&&) noexcept = default; // NOLINT
        GroundDecelerationRequest& operator=(GroundDecelerationRequest&&) noexcept = default; // NOLINT
        ~GroundDecelerationRequest() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundDecelerationRequest& groundDeceleration(const float &v) noexcept;
        float groundDeceleration() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("groundDeceleration"s), m_groundDeceleration, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("groundDeceleration"s), m_groundDeceleration, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_groundDeceleration{ 0.0f }; // field identifier = 1.
        
};
}}

template<>
struct isVisitable<opendlv::proxy::GroundDecelerationRequest> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::proxy::GroundDecelerationRequest> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_SYSTEM_SIGNALSTATUSMESSAGE_HPP
#define OPENDLV_SYSTEM_SIGNALSTATUSMESSAGE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace system {
using namespace std::string_literals; // NOLINT
class LIB_API SignalStatusMessage {
    public:
        SignalStatusMessage() = default;
        SignalStatusMessage(const SignalStatusMessage&) = default;
        SignalStatusMessage& operator=(const SignalStatusMessage&) = default;
        SignalStatusMessage(SignalStatusMessage&&) noexcept = default; // NOLINT
        SignalStatusMessage& operator=(SignalStatusMessage&&) noexcept = default; // NOLINT
        ~SignalStatusMessage() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SignalStatusMessage& code(const int32_t &v) noexcept;
        int32_t code() const noexcept;
        
        SignalStatusMessage& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("int32_t"s), std::move("code"s), m_code, visitor);
            
            doVisit(2, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("int32_t"s), std::move("code"s), m_code, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        int32_t m_code{ 0 }; // field identifier = 1.
        
        std::string m_description{ ""s }; // field identifier = 2.
        
};
}}

template<>
struct isVisitable<opendlv::system::SignalStatusMessage> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::system::SignalStatusMessage> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_SYSTEM_SYSTEMOPERATIONSTATE_HPP
#define OPENDLV_SYSTEM_SYSTEMOPERATIONSTATE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace system {
using namespace std::string_literals; // NOLINT
class LIB_API SystemOperationState {
    public:
        SystemOperationState() = default;
        SystemOperationState(const SystemOperationState&) = default;
        SystemOperationState& operator=(const SystemOperationState&) = default;
        SystemOperationState(SystemOperationState&&) noexcept = default; // NOLINT
        SystemOperationState& operator=(SystemOperationState&&) noexcept = default; // NOLINT
        ~SystemOperationState() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        SystemOperationState& code(const int32_t &v) noexcept;
        int32_t code() const noexcept;
        
        SystemOperationState& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("int32_t"s), std::move("code"s), m_code, visitor);
            
            doVisit(2, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("int32_t"s), std::move("code"s), m_code, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        int32_t m_code{ 0 }; // field identifier = 1.
        
        std::string m_description{ ""s }; // field identifier = 2.
        
};
}}

template<>
struct isVisitable<opendlv::system::SystemOperationState> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::system::SystemOperationState> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_SYSTEM_NETWORKSTATUSMESSAGE_HPP
#define OPENDLV_SYSTEM_NETWORKSTATUSMESSAGE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace system {
using namespace std::string_literals; // NOLINT
class LIB_API NetworkStatusMessage {
    public:
        NetworkStatusMessage() = default;
        NetworkStatusMessage(const NetworkStatusMessage&) = default;
        NetworkStatusMessage& operator=(const NetworkStatusMessage&) = default;
        NetworkStatusMessage(NetworkStatusMessage&&) noexcept = default; // NOLINT
        NetworkStatusMessage& operator=(NetworkStatusMessage&&) noexcept = default; // NOLINT
        ~NetworkStatusMessage() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        NetworkStatusMessage& code(const int32_t &v) noexcept;
        int32_t code() const noexcept;
        
        NetworkStatusMessage& description(const std::string &v) noexcept;
        std::string description() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("int32_t"s), std::move("code"s), m_code, visitor);
            
            doVisit(2, std::move("std::string"s), std::move("description"s), m_description, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("int32_t"s), std::move("code"s), m_code, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("std::string"s), std::move("description"s), m_description, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        int32_t m_code{ 0 }; // field identifier = 1.
        
        std::string m_description{ ""s }; // field identifier = 2.
        
};
}}

template<>
struct isVisitable<opendlv::system::NetworkStatusMessage> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::system::NetworkStatusMessage> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_SENSATION_DIRECTION_HPP
#define OPENDLV_LOGIC_SENSATION_DIRECTION_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace sensation {
using namespace std::string_literals; // NOLINT
class LIB_API Direction {
    public:
        Direction() = default;
        Direction(const Direction&) = default;
        Direction& operator=(const Direction&) = default;
        Direction(Direction&&) noexcept = default; // NOLINT
        Direction& operator=(Direction&&) noexcept = default; // NOLINT
        ~Direction() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Direction& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        Direction& zenithAngle(const float &v) noexcept;
        float zenithAngle() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 1.
        
        float m_zenithAngle{ 0.0f }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::sensation::Direction> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::sensation::Direction> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_SENSATION_POINT_HPP
#define OPENDLV_LOGIC_SENSATION_POINT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace sensation {
using namespace std::string_literals; // NOLINT
class LIB_API Point {
    public:
        Point() = default;
        Point(const Point&) = default;
        Point& operator=(const Point&) = default;
        Point(Point&&) noexcept = default; // NOLINT
        Point& operator=(Point&&) noexcept = default; // NOLINT
        ~Point() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Point& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        Point& zenithAngle(const float &v) noexcept;
        float zenithAngle() const noexcept;
        
        Point& distance(const float &v) noexcept;
        float distance() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, visitor);
            
            doVisit(3, std::move("float"s), std::move("distance"s), m_distance, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("distance"s), m_distance, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 1.
        
        float m_zenithAngle{ 0.0f }; // field identifier = 2.
        
        float m_distance{ 0.0f }; // field identifier = 3.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::sensation::Point> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::sensation::Point> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_SENSATION_GEOLOCATION_HPP
#define OPENDLV_LOGIC_SENSATION_GEOLOCATION_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace sensation {
using namespace std::string_literals; // NOLINT
class LIB_API Geolocation {
    public:
        Geolocation() = default;
        Geolocation(const Geolocation&) = default;
        Geolocation& operator=(const Geolocation&) = default;
        Geolocation(Geolocation&&) noexcept = default; // NOLINT
        Geolocation& operator=(Geolocation&&) noexcept = default; // NOLINT
        ~Geolocation() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Geolocation& latitude(const float &v) noexcept;
        float latitude() const noexcept;
        
        Geolocation& longitude(const float &v) noexcept;
        float longitude() const noexcept;
        
        Geolocation& altitude(const float &v) noexcept;
        float altitude() const noexcept;
        
        Geolocation& heading(const float &v) noexcept;
        float heading() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("latitude"s), m_latitude, visitor);
            
            doVisit(2, std::move("float"s), std::move("longitude"s), m_longitude, visitor);
            
            doVisit(3, std::move("float"s), std::move("altitude"s), m_altitude, visitor);
            
            doVisit(4, std::move("float"s), std::move("heading"s), m_heading, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("latitude"s), m_latitude, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("longitude"s), m_longitude, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("altitude"s), m_altitude, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("heading"s), m_heading, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_latitude{ 0.0f }; // field identifier = 1.
        
        float m_longitude{ 0.0f }; // field identifier = 2.
        
        float m_altitude{ 0.0f }; // field identifier = 3.
        
        float m_heading{ 0.0f }; // field identifier = 4.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::sensation::Geolocation> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::sensation::Geolocation> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_SENSATION_EQUILIBRIOCEPTION_HPP
#define OPENDLV_LOGIC_SENSATION_EQUILIBRIOCEPTION_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace sensation {
using namespace std::string_literals; // NOLINT
class LIB_API Equilibrioception {
    public:
        Equilibrioception() = default;
        Equilibrioception(const Equilibrioception&) = default;
        Equilibrioception& operator=(const Equilibrioception&) = default;
        Equilibrioception(Equilibrioception&&) noexcept = default; // NOLINT
        Equilibrioception& operator=(Equilibrioception&&) noexcept = default; // NOLINT
        ~Equilibrioception() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Equilibrioception& vx(const float &v) noexcept;
        float vx() const noexcept;
        
        Equilibrioception& vy(const float &v) noexcept;
        float vy() const noexcept;
        
        Equilibrioception& vz(const float &v) noexcept;
        float vz() const noexcept;
        
        Equilibrioception& rollRate(const float &v) noexcept;
        float rollRate() const noexcept;
        
        Equilibrioception& pitchRate(const float &v) noexcept;
        float pitchRate() const noexcept;
        
        Equilibrioception& yawRate(const float &v) noexcept;
        float yawRate() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("vx"s), m_vx, visitor);
            
            doVisit(2, std::move("float"s), std::move("vy"s), m_vy, visitor);
            
            doVisit(3, std::move("float"s), std::move("vz"s), m_vz, visitor);
            
            doVisit(4, std::move("float"s), std::move("rollRate"s), m_rollRate, visitor);
            
            doVisit(5, std::move("float"s), std::move("pitchRate"s), m_pitchRate, visitor);
            
            doVisit(6, std::move("float"s), std::move("yawRate"s), m_yawRate, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("vx"s), m_vx, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("vy"s), m_vy, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("vz"s), m_vz, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("rollRate"s), m_rollRate, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("float"s), std::move("pitchRate"s), m_pitchRate, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("yawRate"s), m_yawRate, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_vx{ 0.0f }; // field identifier = 1.
        
        float m_vy{ 0.0f }; // field identifier = 2.
        
        float m_vz{ 0.0f }; // field identifier = 3.
        
        float m_rollRate{ 0.0f }; // field identifier = 4.
        
        float m_pitchRate{ 0.0f }; // field identifier = 5.
        
        float m_yawRate{ 0.0f }; // field identifier = 6.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::sensation::Equilibrioception> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::sensation::Equilibrioception> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECT_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API Object {
    public:
        Object() = default;
        Object(const Object&) = default;
        Object& operator=(const Object&) = default;
        Object(Object&&) noexcept = default; // NOLINT
        Object& operator=(Object&&) noexcept = default; // NOLINT
        ~Object() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        Object& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::Object> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::Object> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECTTYPE_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECTTYPE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API ObjectType {
    public:
        ObjectType() = default;
        ObjectType(const ObjectType&) = default;
        ObjectType& operator=(const ObjectType&) = default;
        ObjectType(ObjectType&&) noexcept = default; // NOLINT
        ObjectType& operator=(ObjectType&&) noexcept = default; // NOLINT
        ~ObjectType() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ObjectType& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        
        ObjectType& type(const uint32_t &v) noexcept;
        uint32_t type() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            doVisit(2, std::move("uint32_t"s), std::move("type"s), m_type, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint32_t"s), std::move("type"s), m_type, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
        uint32_t m_type{ 0 }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::ObjectType> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::ObjectType> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECTPROPERTY_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECTPROPERTY_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API ObjectProperty {
    public:
        ObjectProperty() = default;
        ObjectProperty(const ObjectProperty&) = default;
        ObjectProperty& operator=(const ObjectProperty&) = default;
        ObjectProperty(ObjectProperty&&) noexcept = default; // NOLINT
        ObjectProperty& operator=(ObjectProperty&&) noexcept = default; // NOLINT
        ~ObjectProperty() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ObjectProperty& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        
        ObjectProperty& property(const std::string &v) noexcept;
        std::string property() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            doVisit(2, std::move("std::string"s), std::move("property"s), m_property, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("std::string"s), std::move("property"s), m_property, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
        std::string m_property{ ""s }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::ObjectProperty> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::ObjectProperty> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECTDIRECTION_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECTDIRECTION_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API ObjectDirection {
    public:
        ObjectDirection() = default;
        ObjectDirection(const ObjectDirection&) = default;
        ObjectDirection& operator=(const ObjectDirection&) = default;
        ObjectDirection(ObjectDirection&&) noexcept = default; // NOLINT
        ObjectDirection& operator=(ObjectDirection&&) noexcept = default; // NOLINT
        ~ObjectDirection() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ObjectDirection& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        
        ObjectDirection& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        ObjectDirection& zenithAngle(const float &v) noexcept;
        float zenithAngle() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            doVisit(2, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(3, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 2.
        
        float m_zenithAngle{ 0.0f }; // field identifier = 3.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::ObjectDirection> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::ObjectDirection> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECTDISTANCE_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECTDISTANCE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API ObjectDistance {
    public:
        ObjectDistance() = default;
        ObjectDistance(const ObjectDistance&) = default;
        ObjectDistance& operator=(const ObjectDistance&) = default;
        ObjectDistance(ObjectDistance&&) noexcept = default; // NOLINT
        ObjectDistance& operator=(ObjectDistance&&) noexcept = default; // NOLINT
        ~ObjectDistance() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ObjectDistance& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        
        ObjectDistance& distance(const float &v) noexcept;
        float distance() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            doVisit(2, std::move("float"s), std::move("distance"s), m_distance, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("distance"s), m_distance, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
        float m_distance{ 0.0f }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::ObjectDistance> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::ObjectDistance> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_OBJECTANGULARBLOB_HPP
#define OPENDLV_LOGIC_PERCEPTION_OBJECTANGULARBLOB_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API ObjectAngularBlob {
    public:
        ObjectAngularBlob() = default;
        ObjectAngularBlob(const ObjectAngularBlob&) = default;
        ObjectAngularBlob& operator=(const ObjectAngularBlob&) = default;
        ObjectAngularBlob(ObjectAngularBlob&&) noexcept = default; // NOLINT
        ObjectAngularBlob& operator=(ObjectAngularBlob&&) noexcept = default; // NOLINT
        ~ObjectAngularBlob() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        ObjectAngularBlob& objectId(const uint32_t &v) noexcept;
        uint32_t objectId() const noexcept;
        
        ObjectAngularBlob& width(const float &v) noexcept;
        float width() const noexcept;
        
        ObjectAngularBlob& height(const float &v) noexcept;
        float height() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, visitor);
            
            doVisit(2, std::move("float"s), std::move("width"s), m_width, visitor);
            
            doVisit(3, std::move("float"s), std::move("height"s), m_height, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("objectId"s), m_objectId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("width"s), m_width, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("height"s), m_height, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_objectId{ 0 }; // field identifier = 1.
        
        float m_width{ 0.0f }; // field identifier = 2.
        
        float m_height{ 0.0f }; // field identifier = 3.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::ObjectAngularBlob> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::ObjectAngularBlob> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACE_HPP
#define OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSurface {
    public:
        GroundSurface() = default;
        GroundSurface(const GroundSurface&) = default;
        GroundSurface& operator=(const GroundSurface&) = default;
        GroundSurface(GroundSurface&&) noexcept = default; // NOLINT
        GroundSurface& operator=(GroundSurface&&) noexcept = default; // NOLINT
        ~GroundSurface() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSurface& surfaceId(const uint32_t &v) noexcept;
        uint32_t surfaceId() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_surfaceId{ 0 }; // field identifier = 1.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::GroundSurface> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::GroundSurface> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACETYPE_HPP
#define OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACETYPE_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSurfaceType {
    public:
        GroundSurfaceType() = default;
        GroundSurfaceType(const GroundSurfaceType&) = default;
        GroundSurfaceType& operator=(const GroundSurfaceType&) = default;
        GroundSurfaceType(GroundSurfaceType&&) noexcept = default; // NOLINT
        GroundSurfaceType& operator=(GroundSurfaceType&&) noexcept = default; // NOLINT
        ~GroundSurfaceType() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSurfaceType& surfaceId(const uint32_t &v) noexcept;
        uint32_t surfaceId() const noexcept;
        
        GroundSurfaceType& type(const uint32_t &v) noexcept;
        uint32_t type() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, visitor);
            
            doVisit(2, std::move("uint32_t"s), std::move("type"s), m_type, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("uint32_t"s), std::move("type"s), m_type, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_surfaceId{ 0 }; // field identifier = 1.
        
        uint32_t m_type{ 0 }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::GroundSurfaceType> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::GroundSurfaceType> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACEPROPERTY_HPP
#define OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACEPROPERTY_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSurfaceProperty {
    public:
        GroundSurfaceProperty() = default;
        GroundSurfaceProperty(const GroundSurfaceProperty&) = default;
        GroundSurfaceProperty& operator=(const GroundSurfaceProperty&) = default;
        GroundSurfaceProperty(GroundSurfaceProperty&&) noexcept = default; // NOLINT
        GroundSurfaceProperty& operator=(GroundSurfaceProperty&&) noexcept = default; // NOLINT
        ~GroundSurfaceProperty() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSurfaceProperty& surfaceId(const uint32_t &v) noexcept;
        uint32_t surfaceId() const noexcept;
        
        GroundSurfaceProperty& property(const std::string &v) noexcept;
        std::string property() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, visitor);
            
            doVisit(2, std::move("std::string"s), std::move("property"s), m_property, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("std::string"s), std::move("property"s), m_property, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_surfaceId{ 0 }; // field identifier = 1.
        
        std::string m_property{ ""s }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::GroundSurfaceProperty> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::GroundSurfaceProperty> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACEAREA_HPP
#define OPENDLV_LOGIC_PERCEPTION_GROUNDSURFACEAREA_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace perception {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSurfaceArea {
    public:
        GroundSurfaceArea() = default;
        GroundSurfaceArea(const GroundSurfaceArea&) = default;
        GroundSurfaceArea& operator=(const GroundSurfaceArea&) = default;
        GroundSurfaceArea(GroundSurfaceArea&&) noexcept = default; // NOLINT
        GroundSurfaceArea& operator=(GroundSurfaceArea&&) noexcept = default; // NOLINT
        ~GroundSurfaceArea() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSurfaceArea& surfaceId(const uint32_t &v) noexcept;
        uint32_t surfaceId() const noexcept;
        
        GroundSurfaceArea& leftPerpendicularAngle(const float &v) noexcept;
        float leftPerpendicularAngle() const noexcept;
        
        GroundSurfaceArea& leftDistance(const float &v) noexcept;
        float leftDistance() const noexcept;
        
        GroundSurfaceArea& rightPerpendicularAngle(const float &v) noexcept;
        float rightPerpendicularAngle() const noexcept;
        
        GroundSurfaceArea& rightDistance(const float &v) noexcept;
        float rightDistance() const noexcept;
        
        GroundSurfaceArea& nearPerpendicularAngle(const float &v) noexcept;
        float nearPerpendicularAngle() const noexcept;
        
        GroundSurfaceArea& nearDistance(const float &v) noexcept;
        float nearDistance() const noexcept;
        
        GroundSurfaceArea& farPerpendicularAngle(const float &v) noexcept;
        float farPerpendicularAngle() const noexcept;
        
        GroundSurfaceArea& farDistance(const float &v) noexcept;
        float farDistance() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, visitor);
            
            doVisit(2, std::move("float"s), std::move("leftPerpendicularAngle"s), m_leftPerpendicularAngle, visitor);
            
            doVisit(3, std::move("float"s), std::move("leftDistance"s), m_leftDistance, visitor);
            
            doVisit(4, std::move("float"s), std::move("rightPerpendicularAngle"s), m_rightPerpendicularAngle, visitor);
            
            doVisit(5, std::move("float"s), std::move("rightDistance"s), m_rightDistance, visitor);
            
            doVisit(6, std::move("float"s), std::move("nearPerpendicularAngle"s), m_nearPerpendicularAngle, visitor);
            
            doVisit(7, std::move("float"s), std::move("nearDistance"s), m_nearDistance, visitor);
            
            doVisit(8, std::move("float"s), std::move("farPerpendicularAngle"s), m_farPerpendicularAngle, visitor);
            
            doVisit(9, std::move("float"s), std::move("farDistance"s), m_farDistance, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("uint32_t"s), std::move("surfaceId"s), m_surfaceId, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("leftPerpendicularAngle"s), m_leftPerpendicularAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("leftDistance"s), m_leftDistance, preVisit, visit, postVisit);
            
            doTripletForwardVisit(4, std::move("float"s), std::move("rightPerpendicularAngle"s), m_rightPerpendicularAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(5, std::move("float"s), std::move("rightDistance"s), m_rightDistance, preVisit, visit, postVisit);
            
            doTripletForwardVisit(6, std::move("float"s), std::move("nearPerpendicularAngle"s), m_nearPerpendicularAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(7, std::move("float"s), std::move("nearDistance"s), m_nearDistance, preVisit, visit, postVisit);
            
            doTripletForwardVisit(8, std::move("float"s), std::move("farPerpendicularAngle"s), m_farPerpendicularAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(9, std::move("float"s), std::move("farDistance"s), m_farDistance, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        uint32_t m_surfaceId{ 0 }; // field identifier = 1.
        
        float m_leftPerpendicularAngle{ 0.0f }; // field identifier = 2.
        
        float m_leftDistance{ 0.0f }; // field identifier = 3.
        
        float m_rightPerpendicularAngle{ 0.0f }; // field identifier = 4.
        
        float m_rightDistance{ 0.0f }; // field identifier = 5.
        
        float m_nearPerpendicularAngle{ 0.0f }; // field identifier = 6.
        
        float m_nearDistance{ 0.0f }; // field identifier = 7.
        
        float m_farPerpendicularAngle{ 0.0f }; // field identifier = 8.
        
        float m_farDistance{ 0.0f }; // field identifier = 9.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::perception::GroundSurfaceArea> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::perception::GroundSurfaceArea> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_ACTION_AIMDIRECTION_HPP
#define OPENDLV_LOGIC_ACTION_AIMDIRECTION_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace action {
using namespace std::string_literals; // NOLINT
class LIB_API AimDirection {
    public:
        AimDirection() = default;
        AimDirection(const AimDirection&) = default;
        AimDirection& operator=(const AimDirection&) = default;
        AimDirection(AimDirection&&) noexcept = default; // NOLINT
        AimDirection& operator=(AimDirection&&) noexcept = default; // NOLINT
        ~AimDirection() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AimDirection& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        AimDirection& zentihAngle(const float &v) noexcept;
        float zentihAngle() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(2, std::move("float"s), std::move("zentihAngle"s), m_zentihAngle, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("zentihAngle"s), m_zentihAngle, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 1.
        
        float m_zentihAngle{ 0.0f }; // field identifier = 2.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::action::AimDirection> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::action::AimDirection> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_ACTION_AIMPOINT_HPP
#define OPENDLV_LOGIC_ACTION_AIMPOINT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace action {
using namespace std::string_literals; // NOLINT
class LIB_API AimPoint {
    public:
        AimPoint() = default;
        AimPoint(const AimPoint&) = default;
        AimPoint& operator=(const AimPoint&) = default;
        AimPoint(AimPoint&&) noexcept = default; // NOLINT
        AimPoint& operator=(AimPoint&&) noexcept = default; // NOLINT
        ~AimPoint() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        AimPoint& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        AimPoint& zenithAngle(const float &v) noexcept;
        float zenithAngle() const noexcept;
        
        AimPoint& distance(const float &v) noexcept;
        float distance() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, visitor);
            
            doVisit(3, std::move("float"s), std::move("distance"s), m_distance, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("distance"s), m_distance, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 1.
        
        float m_zenithAngle{ 0.0f }; // field identifier = 2.
        
        float m_distance{ 0.0f }; // field identifier = 3.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::action::AimPoint> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::action::AimPoint> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_ACTION_PREVIEWPOINT_HPP
#define OPENDLV_LOGIC_ACTION_PREVIEWPOINT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace action {
using namespace std::string_literals; // NOLINT
class LIB_API PreviewPoint {
    public:
        PreviewPoint() = default;
        PreviewPoint(const PreviewPoint&) = default;
        PreviewPoint& operator=(const PreviewPoint&) = default;
        PreviewPoint(PreviewPoint&&) noexcept = default; // NOLINT
        PreviewPoint& operator=(PreviewPoint&&) noexcept = default; // NOLINT
        ~PreviewPoint() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        PreviewPoint& azimuthAngle(const float &v) noexcept;
        float azimuthAngle() const noexcept;
        
        PreviewPoint& zenithAngle(const float &v) noexcept;
        float zenithAngle() const noexcept;
        
        PreviewPoint& distance(const float &v) noexcept;
        float distance() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, visitor);
            
            doVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, visitor);
            
            doVisit(3, std::move("float"s), std::move("distance"s), m_distance, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("azimuthAngle"s), m_azimuthAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(2, std::move("float"s), std::move("zenithAngle"s), m_zenithAngle, preVisit, visit, postVisit);
            
            doTripletForwardVisit(3, std::move("float"s), std::move("distance"s), m_distance, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_azimuthAngle{ 0.0f }; // field identifier = 1.
        
        float m_zenithAngle{ 0.0f }; // field identifier = 2.
        
        float m_distance{ 0.0f }; // field identifier = 3.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::action::PreviewPoint> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::action::PreviewPoint> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_COGNITION_GROUNDSTEERINGLIMIT_HPP
#define OPENDLV_LOGIC_COGNITION_GROUNDSTEERINGLIMIT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace cognition {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSteeringLimit {
    public:
        GroundSteeringLimit() = default;
        GroundSteeringLimit(const GroundSteeringLimit&) = default;
        GroundSteeringLimit& operator=(const GroundSteeringLimit&) = default;
        GroundSteeringLimit(GroundSteeringLimit&&) noexcept = default; // NOLINT
        GroundSteeringLimit& operator=(GroundSteeringLimit&&) noexcept = default; // NOLINT
        ~GroundSteeringLimit() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSteeringLimit& steeringLimit(const float &v) noexcept;
        float steeringLimit() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("steeringLimit"s), m_steeringLimit, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("steeringLimit"s), m_steeringLimit, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_steeringLimit{ 0.0f }; // field identifier = 1.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::cognition::GroundSteeringLimit> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::cognition::GroundSteeringLimit> {
    static const bool value = true;
};
#endif


/*
 * THIS IS AN AUTO-GENERATED FILE. DO NOT MODIFY AS CHANGES MIGHT BE OVERWRITTEN!
 */

#ifndef VISITABLE_TYPE_TRAIT
#define VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct visitorSelector {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct visitorSelector<true> {
    template<typename T, class Visitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
        visitor.visit(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<typename T>
struct isVisitable {
    static const bool value = false;
};

template<typename T, class Visitor>
void doVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, Visitor &visitor) {
    visitorSelector<isVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, visitor);
}
#endif

#ifndef TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#define TRIPLET_FORWARD_VISITABLE_TYPE_TRAIT
#include <cstdint>
#include <string>
#include <utility>

template<bool b>
struct tripletForwardVisitorSelector {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)preVisit;
        (void)postVisit;
        std::forward<Visitor>(visit)(fieldIdentifier, std::move(typeName), std::move(name), value);
    }
};

template<>
struct tripletForwardVisitorSelector<true> {
    template<typename T, class PreVisitor, class Visitor, class PostVisitor>
    static void impl(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
        (void)fieldIdentifier;
        (void)typeName;
        (void)name;
        // Apply preVisit, visit, and postVisit on value.
        value.accept(preVisit, visit, postVisit);
    }
};

template<typename T>
struct isTripletForwardVisitable {
    static const bool value = false;
};

template< typename T, class PreVisitor, class Visitor, class PostVisitor>
void doTripletForwardVisit(uint32_t fieldIdentifier, std::string &&typeName, std::string &&name, T &value, PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
    tripletForwardVisitorSelector<isTripletForwardVisitable<T>::value >::impl(fieldIdentifier, std::move(typeName), std::move(name), value, std::move(preVisit), std::move(visit), std::move(postVisit)); // NOLINT
}
#endif


#ifndef OPENDLV_LOGIC_COGNITION_GROUNDSPEEDLIMIT_HPP
#define OPENDLV_LOGIC_COGNITION_GROUNDSPEEDLIMIT_HPP

#ifdef WIN32
    // Export symbols if compile flags "LIB_SHARED" and "LIB_EXPORTS" are set on Windows.
    #ifdef LIB_SHARED
        #ifdef LIB_EXPORTS
            #define LIB_API __declspec(dllexport)
        #else
            #define LIB_API __declspec(dllimport)
        #endif
    #else
        // Disable definition if linking statically.
        #define LIB_API
    #endif
#else
    // Disable definition for non-Win32 systems.
    #define LIB_API
#endif

#include <string>
#include <utility>
namespace opendlv { namespace logic { namespace cognition {
using namespace std::string_literals; // NOLINT
class LIB_API GroundSpeedLimit {
    public:
        GroundSpeedLimit() = default;
        GroundSpeedLimit(const GroundSpeedLimit&) = default;
        GroundSpeedLimit& operator=(const GroundSpeedLimit&) = default;
        GroundSpeedLimit(GroundSpeedLimit&&) noexcept = default; // NOLINT
        GroundSpeedLimit& operator=(GroundSpeedLimit&&) noexcept = default; // NOLINT
        ~GroundSpeedLimit() = default;

    public:
        static int32_t ID();
        static const std::string ShortName();
        static const std::string LongName();
        
        GroundSpeedLimit& speedLimit(const float &v) noexcept;
        float speedLimit() const noexcept;
        

        template<class Visitor>
        void accept(Visitor &visitor) {
            visitor.preVisit(ID(), ShortName(), LongName());
            
            doVisit(1, std::move("float"s), std::move("speedLimit"s), m_speedLimit, visitor);
            
            visitor.postVisit();
        }

        template<class PreVisitor, class Visitor, class PostVisitor>
        void accept(PreVisitor &&preVisit, Visitor &&visit, PostVisitor &&postVisit) {
            std::forward<PreVisitor>(preVisit)(ID(), ShortName(), LongName());
            
            doTripletForwardVisit(1, std::move("float"s), std::move("speedLimit"s), m_speedLimit, preVisit, visit, postVisit);
            
            std::forward<PostVisitor>(postVisit)();
        }

    private:
        
        float m_speedLimit{ 0.0f }; // field identifier = 1.
        
};
}}}

template<>
struct isVisitable<opendlv::logic::cognition::GroundSpeedLimit> {
    static const bool value = true;
};
template<>
struct isTripletForwardVisitable<opendlv::logic::cognition::GroundSpeedLimit> {
    static const bool value = true;
};
#endif

