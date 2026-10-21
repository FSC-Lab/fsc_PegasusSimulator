#!/bin/bash
if [ "$1" = "-C" ]; then echo "$(date +%s.%N) CANCEL" >> "$FAKE_AUDIO_LOG"; exit 0; fi
shift
text="$*"
echo "$(date +%s.%N) SAY-START $text" >> "$FAKE_AUDIO_LOG"
sleep "$(awk -v n="$(wc -w <<< "$text")" 'BEGIN{print 0.3*n}')"
echo "$(date +%s.%N) SAY-END $text" >> "$FAKE_AUDIO_LOG"
