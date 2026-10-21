#!/bin/bash
echo "$(date +%s.%N) SIREN-START" >> "$FAKE_AUDIO_LOG"
cat > /dev/null
sleep 1.2
echo "$(date +%s.%N) SIREN-END" >> "$FAKE_AUDIO_LOG"
