#ifndef INOCOMPAT_H
	#define INOCOMPAT_H
	
	
	// =============================================================================
	// ArduinoCompat.h
	// Provides all Arduino/Teensy APIs so embedded source files compile on PC
	// without modification.
	//
	// KEY FEATURE — pin interception:
	//   StateMachine.cpp calls analogWrite(p.PWM1, speed) and digitalWrite(p.INB1, HIGH)
	//   directly. Rather than letting those be no-ops, we route them through a
	//   global DataLayer pointer so the sim captures the real motor commands.
	//
	//   AGVController calls ArduinoCompat::SetDataLayer(&dataLayer) before
	//   invoking any StateMachine methods. After that, every analogWrite /
	//   digitalWrite from StateMachine lands in the correct DataLayer field.
	//   StateMachine.cpp and AGVPins.h are completely unchanged.
	// =============================================================================
	
	#include <cstdint>
	#include <cmath>
	#include <chrono>
	#include <iostream>
	#include <random>
	#include <algorithm>
	
	// Forward-declare so we can hold a pointer without a circular include.
	// datalayer.h is included after the function stubs below where it is needed.
	struct DataLayer;
	
	namespace ArduinoCompat {
	    // AGVController::Update sets this before calling StateMachine methods.
	    inline DataLayer* g_dataLayer = nullptr;
	    inline void SetDataLayer(DataLayer* dl) { g_dataLayer = dl; }
		inline unsigned long simTimeOffset = 0;
        }
	
	// Pin numbers — must match AGVPins.h exactly
	namespace AGVPinMap {
	    constexpr int PWM1 = 9;
	    constexpr int PWM2 = 6;
	    constexpr int INB1 = 8;
	    constexpr int INB2 = 5;
	}
	
	// ---------------------------------------------------------------------------
	// Constants
	// ---------------------------------------------------------------------------
	#ifndef HIGH
	#define HIGH 1
	#endif
	#ifndef LOW
	#define LOW  0
	#endif
	#ifndef OUTPUT
	#define OUTPUT 1
	#endif
	#ifndef INPUT
	#define INPUT  0
	#endif
	#ifndef M_PI
	#define M_PI 3.14159265358979323846
    #endif
    #ifndef HEX
    #define HEX 16
    #endif
	// ---------------------------------------------------------------------------
	// Timing
	// ---------------------------------------------------------------------------
	inline unsigned long millis() {
    static auto start = std::chrono::steady_clock::now();
    return (unsigned long)std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - start)
        .count();
    }
	
	inline unsigned long micros() {
	    static auto start = std::chrono::steady_clock::now();
	    return (unsigned long)std::chrono::duration_cast<std::chrono::microseconds>(
	        std::chrono::steady_clock::now() - start).count();
	}
	
	inline void delay(unsigned long ms) {}
	
	// ---------------------------------------------------------------------------
	// Math
	// ---------------------------------------------------------------------------
    #define constrain(val, lo, hi) ((val) < (lo) ? (lo) : ((val) > (hi) ? (hi) : (val)))
	
	// ---------------------------------------------------------------------------
	// Random
	// ---------------------------------------------------------------------------
	namespace _rng {
	    inline std::mt19937& engine() {
	        static std::mt19937 e{std::random_device{}()};
	        return e;
	    }
	}
	inline long random(long max_val) {
	    if (max_val <= 0) return 0;
	    return std::uniform_int_distribution<long>(0, max_val - 1)(_rng::engine());
	}
	inline long random(long min_val, long max_val) {
	    if (max_val <= min_val) return min_val;
	    return std::uniform_int_distribution<long>(min_val, max_val - 1)(_rng::engine());
	}
	// ---------------------------------------------------------------------------
	// HardwareSerial stub
	// ---------------------------------------------------------------------------
	class HardwareSerial {
	public:
	    void begin(unsigned long) {}
	    void end() {}
	    int  available()              { return 0; }
	    int  read()                   { return -1; }
	    void readBytes(uint8_t*, int) {}
	    void write(const uint8_t*, size_t) {}
	    void write(uint8_t) {}
	    void flush() {}
	};


	// ---------------------------------------------------------------------------
	// Serial
	// ---------------------------------------------------------------------------
	class _SerialClass : public HardwareSerial {
	public:
	    void begin(unsigned long) {}
	    template<typename T> void print(const T& v)   { std::cout << v; }
	    void print(uint8_t v, int base)  { if (base==16) std::cout<<std::hex<<(int)v<<std::dec; else std::cout<<(int)v; }
	    void print(int v, int base)      { if (base==16) std::cout<<std::hex<<v<<std::dec; else std::cout<<v; }
	    template<typename T> void println(const T& v) { std::cout << v << '\n'; }
	    void println(uint8_t v, int base){ if (base==16) std::cout<<std::hex<<(int)v<<std::dec<<'\n'; else std::cout<<(int)v<<'\n'; }
	    void println(int v, int base)    { if (base==16) std::cout<<std::hex<<v<<std::dec<<'\n'; else std::cout<<v<<'\n'; }
	    void println() { std::cout << '\n'; }
	    int  available() { return 0; }
	    void flush() {}
	};
	inline _SerialClass Serial;
	inline _SerialClass Serial1;
	inline _SerialClass Serial2;
	
	// ---------------------------------------------------------------------------
	// pinMode — no-op
	// ---------------------------------------------------------------------------
	inline void pinMode(int, int) {}
	
	// ---------------------------------------------------------------------------
	// analogWrite and digitalWrite — must include datalayer.h here so we can
	// write into DataLayer fields. This is safe because datalayer.h has no
	// Arduino dependencies of its own.
	// ---------------------------------------------------------------------------
	#include "datalayer.h"
	
	inline void analogWrite(int pin, int value) {
	    if (!ArduinoCompat::g_dataLayer) return;
	    DataLayer& dl = *ArduinoCompat::g_dataLayer;
	    if      (pin == AGVPinMap::PWM1) dl.leftMotor.PWM  = value;
	    else if (pin == AGVPinMap::PWM2) dl.rightMotor.PWM = value;
	}
	
	inline void digitalWrite(int pin, int value) {
	    if (!ArduinoCompat::g_dataLayer) return;
	    DataLayer& dl = *ArduinoCompat::g_dataLayer;
	    if      (pin == AGVPinMap::INB1) dl.leftMotor.direction  = (value == HIGH) ? FORWARD : REVERSE;
	    else if (pin == AGVPinMap::INB2) dl.rightMotor.direction = (value == HIGH) ? FORWARD : REVERSE;
	}
	
	// ---------------------------------------------------------------------------
	// Wire stub
	// ---------------------------------------------------------------------------
	class _WireClass {
	public:
	    void begin() {}
	    void setClock(uint32_t) {}
	};
	inline _WireClass Wire;

	
	
	
	#endif
