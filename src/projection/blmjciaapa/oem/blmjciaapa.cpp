// Self-resolving thunks for the OEM functions inside blmjciaapa.so.
// See blmjciaapa.h for the rationale and the anchor-and-offset model.
//
// Each public function is a thin trampoline that, on first call,
// computes its target address (load bias of blmjciaapa.so + the
// target's file offset), caches it, and forwards. The load bias comes
// from the one dynamically-exported anchor symbol, GetServiceInterfaces,
// via dlsym. No init step, no shared global state, no ordering rule.
// An unresolved target (blmjciaapa.so not mapped — i.e. not the
// {L_jciAAPA} PID) degrades to a benign failure value.

#define LOG_TAG "BLMJCIAAPA"
#include "../log.h"

#include "blmjciaapa.h"
#include "projection.h"

#include <dlfcn.h>
#include <atomic>
#include <pthread.h>

namespace {

// All firmware-bound addresses live in the selected profile.
namespace off = fw74_00_324a::blmjciaapa;

// Load bias of the already-mapped blmjciaapa.so: address of the
// exported anchor minus its file offset. 0 until resolved (or if
// blmjciaapa.so isn't mapped). Retries until it succeeds — the BLM's
// AapProc singleton may not be up on the earliest calls, but the
// library itself is mapped for the whole {L_jciAAPA} PID lifetime, so
// once non-zero the bias never changes.
uintptr_t blm_base()
{
	// Only unresolved thunks reach this path. Keep dlopen/cache publication
	// serialized while touch, pairing and mute workers resolve their first call.
	static pthread_mutex_t mu = PTHREAD_MUTEX_INITIALIZER;
	pthread_mutex_lock(&mu);
	static uintptr_t base = 0;
	if (base == 0) {
		void *handle = dlopen("/jci/aapa/blmjciaapa.so",
		                      RTLD_NOW | RTLD_NOLOAD);
		void *sym = handle ? dlsym(handle, "GetServiceInterfaces")
		                   : nullptr;
		if (sym) {
			base = reinterpret_cast<uintptr_t>(sym)
			       - off::GetServiceInterfaces;
		} else {
			if (handle) dlclose(handle);
			// Log once — repeated misses (e.g. we're not the
			// {L_jciAAPA} PID, so blmjciaapa.so is never mapped)
			// would otherwise spam the per-frame touch path.
			static bool logged = false;
			if (!logged) {
				logged = true;
				LOGC("oem: blmjciaapa.so anchor GetServiceInterfaces "
				     "unresolved (not the L_jciAAPA PID?)");
			}
		}
	}
	const uintptr_t result = base;
	pthread_mutex_unlock(&mu);
	return result;
}

// Resolve one offset-based target to an absolute address, or nullptr
// if the load bias isn't available yet.
void *blm_addr(uintptr_t target_offset)
{
	uintptr_t base = blm_base();
	return base ? reinterpret_cast<void *>(base + target_offset) : nullptr;
}

} // namespace

// Define a self-resolving thunk that forwards to the OEM target at
// off::<name>. `params` / `args` are parenthesised lists.
#define BLM_THUNK(ret, name, params, args, failval) \
	ret name params \
	{ \
		using Fn = ret (*) params; \
		static std::atomic<Fn> cached{nullptr}; \
		Fn fn = cached.load(std::memory_order_acquire); \
		if (!fn) { \
			fn = reinterpret_cast<Fn>(blm_addr(off::name)); \
			if (fn) cached.store(fn, std::memory_order_release); \
		} \
		return fn ? fn args : (failval); \
	}

// Void-returning variant of BLM_THUNK (the value-returning macro can't
// express a void forward). Same self-resolving-and-caching model.
#define BLM_THUNK_VOID(name, params, args) \
	void name params \
	{ \
		using Fn = void (*) params; \
		static std::atomic<Fn> cached{nullptr}; \
		Fn fn = cached.load(std::memory_order_acquire); \
		if (!fn) { \
			fn = reinterpret_cast<Fn>(blm_addr(off::name)); \
			if (fn) cached.store(fn, std::memory_order_release); \
		} \
		if (fn) fn args; \
	}

// Read a scalar field at a fixed byte offset within an OEM instance the
// caller already holds. Pure pointer arithmetic — unlike BLM_THUNK it
// needs no load-bias resolution, since the instance pointer is supplied.
// A null instance degrades to `failval`.
#define BLM_FIELD(ret, name, self_off, failval) \
	ret name(void *self) \
	{ \
		return self \
		    ? *reinterpret_cast<ret *>( \
		          reinterpret_cast<char *>(self) + (self_off)) \
		    : (failval); \
	}

// Variant returning a pointer INTO the instance, for an inline buffer /
// char array that is addressed rather than dereferenced. Null -> nullptr.
#define BLM_FIELD_PTR(elem, name, self_off) \
	const elem *name(void *self) \
	{ \
		return self \
		    ? reinterpret_cast<const elem *>( \
		          reinterpret_cast<char *>(self) + (self_off)) \
		    : nullptr; \
	}

BLM_THUNK(void *, Singleton_AapProc_GetInstance, (void), (), nullptr)
BLM_THUNK(void *, AapProc_GetVideoManager, (void *self), (self), nullptr)
BLM_THUNK(void *, AapProc_GetRaceAap, (void *self), (self), nullptr)
BLM_THUNK(void *, AapProc_GetAudioManager, (void *self), (self), nullptr)
BLM_THUNK(void *, AapProc_GetAapConnectionManager, (void *self), (self), nullptr)
BLM_THUNK(int,    VideoManager_IsAAVideoInFocus, (void *self), (self), 0)
BLM_THUNK(int,    AudioManager_IsAAMediaInFocus, (void *self), (self), 0)
BLM_THUNK(int,    AudioManager_IsAAMediaInPlaying, (void *self), (self), 0)
BLM_THUNK(int,    RaceAap_SendTouchInput,
          (void *self, AAP_TouchEvent *evt), (self, evt), -1)
BLM_THUNK(int,    RaceAap_SendKeyInput,
          (void *self, AAP_KeyEvent *evt), (self, evt), -1)
BLM_THUNK_VOID(AapConnectionManager_NotifyBtPairingResult,
               (void *self, int result), (self, result))
BLM_THUNK_VOID(AapConnectionManager_ActivateAapSession,
               (void *self), (self))

BLM_FIELD_PTR(char, AapConnectionManager_dev_name,
              off::AapConnectionManager_DevName)
BLM_FIELD(int, AapConnectionManager_connect_mode,
          off::AapConnectionManager_ConnectMode, -1)

		  
// AapConnectionManager instance accessor. GetInstance() may return
// nullptr before the AapProc singleton is up (or when we're in the
// wrong PID); callers must tolerate the nullptr degradation. Forwards
// through the OEM AapProc::GetAapConnectionManager getter (thunk above).
void *AapConnectionManager_instance(void)
{
	void *aap = Singleton_AapProc_GetInstance();
	return aap ? AapProc_GetAapConnectionManager(aap) : nullptr;
}
