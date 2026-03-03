#pragma once
#ifdef __cplusplus
extern "C" {
#endif
void audio_init(void);
void audio_play_file(const char *path);
void audio_set_volume(int vol);
void audio_stop(void);
#ifdef __cplusplus
}
#endif
