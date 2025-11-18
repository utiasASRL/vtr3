// src/components/tools/VoiceControl.js
import { useEffect, useRef } from "react";

const VoiceControl = ({ onVoiceCommand, onAddWaypoint, onCancelCurrentGoal, enabled }) => {
  const recogRef = useRef(null);
  const runningRef = useRef(false); // tracks whether recognition is active

  useEffect(() => {
    console.log("[VoiceControl] useEffect fired, enabled =", enabled);

    const SpeechRecognition = window.SpeechRecognition || window.webkitSpeechRecognition;
    if (!SpeechRecognition) {
      console.warn("[VoiceControl] Browser does not support SpeechRecognition");
      return;
    }

    const recog = new SpeechRecognition();
    recog.continuous = true;
    recog.interimResults = false;
    recog.lang = "en-US";

    recog.onstart = () => {
      runningRef.current = true;
      console.log("[VoiceControl] SpeechRecognition started");
    };

    recog.onspeechstart = () => {
      console.log("[VoiceControl] Speech detected");
    };

    recog.onresult = (event) => {
      const lastResultIndex = event.results.length - 1;
      const transcript = event.results[lastResultIndex][0].transcript.trim();
      console.log(`[VoiceControl] 🗣️ Final Transcript: "${transcript}"`);
      const lower = transcript.toLowerCase();


      // 1) “robot add waypoint as <name>”
      const mAdd = lower.match(/\brobots?\s+add\s+waypoint\s+as\s+(.+)/);
      if (mAdd) {
        console.log(`[VoiceControl] Matched add-waypoint, name = "${mAdd[1]}"`);
        onAddWaypoint(mAdd[1]);
        return;
      }

      // 2) “cancel the current goal”
      const mCancel = lower.match(/\bcancel (?:the\s+)?current goal\b/);
      if (mCancel) {
        console.log(`[VoiceControl] Matched cancel-current-goal`);
        if (onCancelCurrentGoal) onCancelCurrentGoal();
        return;
      }

      // 1) Define each part on its own line, with inline comments
      const GREETING    = `(?:(?:hey|hi|hello|greeting|greetings)[, ]+)?`;                     // optional “hey/hi,”
      const ROBOT       = `\\brobots?[, ]*`;                          // “robot” or “robots” + comma/space
      const POLITE      = `(?:(?:can you|could you|please|could you please|can you please)[, ]+)?`;   // optional politeness
      const LET         = `(?:let's|let us|let me)?\\s*`; // optional "let's/let us/let me"
      const ACTION      = `(?:go to|goto|navigate to|head to|proceed to|move to|go back to|go back)\\s+`; // all verbs
      const ARTICLE     = `(?:(?:the|a|an)\\s+)?`;                    // optional “the/a/an”
      const TARGET      = `(.+)`;                                     // capture the rest as the goal
      const END_ANCHOR  = `$`;                                        // end of string

      // 2) Join them into one pattern string
      const pattern = [
        `^`,          // start of string
        GREETING,
        ROBOT,
        POLITE,
        LET,
        ACTION,
        ARTICLE,
        TARGET,
        END_ANCHOR
      ].join('');

      // 3) Create your RegExp from that string
      const mGo = lower.match(new RegExp(pattern));  // 'i' = case-insensitive if you like


      if (mGo) {
        console.log(`[VoiceControl] Matched go-to, goalName = "${mGo[1]}"`);
        onVoiceCommand(mGo[1]);
      }
    };

    recog.onerror = (e) => {
      console.error("[VoiceControl] SpeechRecognition error:", e.error);
      if (enabled && e.error !== "not-allowed" && e.error !== "service-not-allowed") {
        setTimeout(() => {
          if (!runningRef.current) {
            try { recog.start(); }
            catch (err) { console.error("[VoiceControl] Restart failed:", err); }
          }
        }, 300);
      }
    };

    recog.onend = () => {
      runningRef.current = false;
      console.log("[VoiceControl] SpeechRecognition ended");
      if (enabled) {
        setTimeout(() => {
          if (!runningRef.current) {
            try { recog.start(); }
            catch (err) { console.error("[VoiceControl] Restart on end failed:", err); }
          }
        }, 300);
      }
    };

    recogRef.current = recog;

    if (enabled && !runningRef.current) {
      console.log("[VoiceControl] Starting recognition");
      recog.start();
    } else {
      console.log(
        "[VoiceControl] Recognition not started—enabled:",
        enabled,
        "running:",
        runningRef.current
      );
    }

    return () => {
      console.log("[VoiceControl] Cleaning up, stopping recognition");
      recogRef.current?.stop();
      recogRef.current = null;
      runningRef.current = false;
    };
  }, [onVoiceCommand, onAddWaypoint, enabled]);

  return null;
};

export default VoiceControl;
