import React, { useState, useEffect, useRef } from 'react';
import styled, { keyframes } from 'styled-components';

const API_BASE = 'http://localhost:8000';

// Futuristic neon colors
const neonBlue = '#00eaff';
const neonPurple = '#a259ff';
const darkBg = '#181c24';
const cardBg = '#23283a';
const accent = '#1f2937';
const font = '"Orbitron", "Segoe UI", Arial, sans-serif';

const fadeIn = keyframes`
  from { opacity: 0; transform: translateY(20px); }
  to { opacity: 1; transform: none; }
`;

const Container = styled.div`
  font-family: ${font};
  background: linear-gradient(135deg, #181c24 60%, #23283a 100%);
  min-height: 100vh;
  color: #eaf6fb;
  padding: 0;
`;
const Header = styled.header`
  text-align: center;
  padding: 2rem 0 1rem 0;
  font-size: 2.5rem;
  letter-spacing: 0.1em;
  color: ${neonBlue};
  text-shadow: 0 0 8px ${neonBlue}, 0 0 24px ${neonPurple};
`;
const Main = styled.main`
  display: flex;
  gap: 2rem;
  justify-content: center;
  align-items: flex-start;
  padding: 0 2vw 2vw 2vw;
  @media (max-width: 900px) {
    flex-direction: column;
    align-items: stretch;
  }
`;
const Card = styled.div`
  background: ${cardBg};
  border-radius: 1.5rem;
  box-shadow: 0 0 24px 2px ${neonBlue}33;
  padding: 2rem 1.5rem;
  flex: 1;
  min-width: 320px;
  margin-bottom: 2rem;
`;
const ImageFrame = styled.div`
  background: ${accent};
  border-radius: 1rem;
  box-shadow: 0 0 16px 2px ${neonPurple}55;
  padding: 1rem;
  margin-bottom: 1.5rem;
  text-align: center;
`;
const FuturisticButton = styled.button`
  background: linear-gradient(90deg, ${neonBlue} 0%, ${neonPurple} 100%);
  color: #fff;
  border: none;
  border-radius: 2rem;
  padding: 0.7rem 2.2rem;
  font-size: 1.1rem;
  font-family: ${font};
  margin: 0.5rem 0.5rem 0.5rem 0;
  box-shadow: 0 0 12px 2px ${neonBlue}55;
  cursor: pointer;
  transition: background 0.2s, box-shadow 0.2s, transform 0.1s;
  &:hover {
    background: linear-gradient(90deg, ${neonPurple} 0%, ${neonBlue} 100%);
    box-shadow: 0 0 24px 4px ${neonPurple}99;
    transform: scale(1.04);
  }
`;
const StatusBar = styled.div`
  display: flex;
  justify-content: space-between;
  align-items: center;
  background: ${accent};
  border-radius: 1rem;
  padding: 0.7rem 2rem;
  margin: 2rem 0 0 0;
  box-shadow: 0 0 8px 1px ${neonBlue}33;
  font-size: 1.1rem;
`;
const ChatContainer = styled.div`
  background: ${accent};
  border-radius: 1rem;
  padding: 1.2rem 1rem 1rem 1rem;
  height: 60vh;
  overflow-y: auto;
  margin-bottom: 1rem;
  box-shadow: 0 0 12px 2px ${neonPurple}33;
`;
const ChatBubble = styled.div`
  max-width: 80%;
  margin-bottom: 1.2rem;
  padding: 1rem 1.3rem;
  border-radius: 1.2rem;
  background: ${({ role }) =>
    role === 'user'
      ? `linear-gradient(90deg, ${neonBlue}55 0%, ${neonPurple}33 100%)`
      : `linear-gradient(90deg, ${neonPurple}55 0%, ${neonBlue}33 100%)`};
  color: #fff;
  align-self: ${({ role }) => (role === 'user' ? 'flex-end' : 'flex-start')};
  box-shadow: 0 0 16px 2px ${neonBlue}22;
  animation: ${fadeIn} 0.5s;
  font-size: 1.08rem;
  position: relative;
`;
const InputRow = styled.div`
  display: flex;
  gap: 1rem;
  margin-bottom: 1rem;
`;
const FuturisticInput = styled.input`
  flex: 1;
  background: #23283a;
  border: 2px solid ${neonBlue};
  border-radius: 2rem;
  color: #fff;
  font-size: 1.1rem;
  padding: 0.7rem 1.2rem;
  outline: none;
  transition: border 0.2s, box-shadow 0.2s;
  box-shadow: 0 0 8px 1px ${neonBlue}33;
  &:focus {
    border: 2px solid ${neonPurple};
    box-shadow: 0 0 16px 2px ${neonPurple}99;
  }
`;
const NeonLabel = styled.span`
  color: ${neonBlue};
  text-shadow: 0 0 8px ${neonBlue}99;
  font-weight: bold;
`;
const MicStatus = styled.span`
  color: ${({ enabled }) => (enabled ? neonBlue : neonPurple)};
  text-shadow: 0 0 8px ${({ enabled }) => (enabled ? neonBlue : neonPurple)}99;
  font-weight: bold;
`;
const Small = styled.span`
  font-size: 0.95em;
  color: #b3c6d1;
`;

function App() {
  const [messages, setMessages] = useState([]);
  const [isListening, setIsListening] = useState(false);
  const [micEnabled, setMicEnabled] = useState(true);
  const [systemInitialized, setSystemInitialized] = useState(false);
  const [userInput, setUserInput] = useState('');
  const [imageExists, setImageExists] = useState(false);
  const [sceneExists, setSceneExists] = useState(false);
  const [micStatus, setMicStatus] = useState('🟢 Enabled');
  const [sceneDescription, setSceneDescription] = useState('');
  const [imageUrl, setImageUrl] = useState(`${API_BASE}/image/?t=${Date.now()}`);
  const audioRef = useRef(null);

  useEffect(() => {
    setSystemInitialized(true);
    fetch(`${API_BASE}/image/`, { method: 'HEAD' })
      .then(res => setImageExists(res.ok))
      .catch(() => setImageExists(false));
    fetch(`${API_BASE}/scene_description/`)
      .then(res => res.json())
      .then(data => {
        setSceneExists(!!data.description);
        setSceneDescription(data.description || '');
      })
      .catch(() => setSceneExists(false));
  }, []);

  const handleSend = async () => {
    if (!userInput.trim()) return;
    const newUserMsg = {
      role: 'user',
      content: userInput,
      timestamp: new Date().toISOString(),
    };
    setMessages(prev => [...prev, newUserMsg]);
    await processUserInput(userInput);
    setUserInput('');
  };

  const handleVoiceInput = async () => {
    setIsListening(true);
    if (!navigator.mediaDevices || !window.MediaRecorder) {
      alert('MediaRecorder not supported in this browser.');
      setIsListening(false);
      return;
    }
    try {
      const stream = await navigator.mediaDevices.getUserMedia({ audio: true });
      const mediaRecorder = new window.MediaRecorder(stream);
      let chunks = [];
      mediaRecorder.ondataavailable = e => chunks.push(e.data);
      mediaRecorder.onstop = async () => {
        const blob = new Blob(chunks, { type: 'audio/wav' });
        const formData = new FormData();
        formData.append('file', blob, 'audio.wav');
        const res = await fetch(`${API_BASE}/asr/`, {
          method: 'POST',
          body: formData,
        });
        const data = await res.json();
        const transcribedText = data.text || '[No transcription]';
        setIsListening(false);
        const newUserMsg = {
          role: 'user',
          content: transcribedText,
          timestamp: new Date().toISOString(),
        };
        setMessages(prev => [...prev, newUserMsg]);
        await processUserInput(transcribedText);
      };
      mediaRecorder.start();
      setTimeout(() => {
        mediaRecorder.stop();
        stream.getTracks().forEach(track => track.stop());
      }, 4000);
    } catch (err) {
      alert('Could not access microphone.');
      setIsListening(false);
    }
  };

  const speakText = async (text) => {
    const formData = new FormData();
    formData.append('text', text);
    const res = await fetch(`${API_BASE}/tts/`, {
      method: 'POST',
      body: formData,
    });
    const audioBlob = await res.blob();
    if (audioRef.current) {
      audioRef.current.src = URL.createObjectURL(audioBlob);
      audioRef.current.play();
    }
  };

  const processUserInput = async (input) => {
    const formData = new FormData();
    formData.append('query', input);
    const routeRes = await fetch(`${API_BASE}/route/`, {
      method: 'POST',
      body: formData,
    });
    const routeData = await routeRes.json();
    let botResponse = '';
    if (routeData.route === 'vision') {
      await fetch(`${API_BASE}/capture_image/`, { method: 'POST' });
      setImageUrl(`${API_BASE}/image/?t=${Date.now()}`);
      const sceneRes = await fetch(`${API_BASE}/scene_description/`);
      const sceneData = await sceneRes.json();
      setSceneDescription(sceneData.description || '');
      botResponse = sceneData.description || "I captured an image but couldn't describe the scene yet.";
    } else {
      const sceneRes = await fetch(`${API_BASE}/scene_description/`);
      const sceneData = await sceneRes.json();
      setSceneDescription(sceneData.description || '');
      const actionForm = new FormData();
      actionForm.append('query', input);
      actionForm.append('scene_desc', sceneData.description || '');
      const actionRes = await fetch(`${API_BASE}/action_response/`, {
        method: 'POST',
        body: actionForm,
      });
      const actionData = await actionRes.json();
      botResponse = actionData.response || '[No response]';
    }
    const newBotMsg = {
      role: 'assistant',
      content: botResponse,
      timestamp: new Date().toISOString(),
    };
    setMessages(prev => [...prev, newBotMsg]);
    await speakText(botResponse);
  };

  const handleRefreshImage = async () => {
    await fetch(`${API_BASE}/capture_image/`, { method: 'POST' });
    setImageUrl(`${API_BASE}/image/?t=${Date.now()}`);
    setImageExists(true);
  };

  const handleClearChat = () => {
    setMessages([]);
  };

  const handleTestSystem = async () => {
    const testResponse = "Hello! I am Jetty the robot! I'm ready to help. What would you like me to do?";
    setMessages(prev => [
      ...prev,
      {
        role: 'assistant',
        content: testResponse,
        timestamp: new Date().toISOString(),
      },
    ]);
    await speakText(testResponse);
  };

  const handleToggleMic = () => {
    setMicEnabled(m => !m);
    setMicStatus(micEnabled ? '🔴 Disabled' : '🟢 Enabled');
  };

  return (
    <Container>
      <Header>
        <span style={{ fontWeight: 900, letterSpacing: '0.15em' }}>🤖 JETTY</span>
        <Small> &nbsp;|&nbsp; Futuristic Robot Assistant</Small>
      </Header>
      <Main>
        <Card>
          <h2 style={{ color: neonPurple, textShadow: `0 0 8px ${neonPurple}` }}>Current View</h2>
          <ImageFrame>
            {imageExists ? (
              <img
                src={imageUrl}
                alt="Current camera view"
                style={{ width: '100%', borderRadius: '0.7rem', boxShadow: `0 0 24px 2px ${neonBlue}55` }}
              />
            ) : (
              <div style={{ padding: 32, color: neonBlue, textShadow: `0 0 8px ${neonBlue}` }}>No image available</div>
            )}
          </ImageFrame>
          <FuturisticButton onClick={handleRefreshImage}>🔄 Refresh Image</FuturisticButton>
          <div style={{ margin: '1.5rem 0 0.5rem 0', color: neonBlue, fontWeight: 700 }}>System Status</div>
          <div>Scene Database: <NeonLabel>{sceneExists ? '✅ Available' : '❌ Not found'}</NeonLabel></div>
          <div>Current Image: <NeonLabel>{imageExists ? '✅ Available' : '❌ Not found'}</NeonLabel></div>
          <div>ASR: <NeonLabel>✅ Ready</NeonLabel></div>
          <div>TTS: <NeonLabel>✅ Ready</NeonLabel></div>
          <div>Router: <NeonLabel>✅ Ready</NeonLabel></div>
          <div>VLM: <NeonLabel>✅ Ready</NeonLabel></div>
        </Card>
        <Card>
          <h2 style={{ color: neonBlue, textShadow: `0 0 8px ${neonBlue}` }}>Chat Interface</h2>
          <ChatContainer>
            {messages.map((msg, idx) => (
              <div key={idx} style={{ display: 'flex', flexDirection: 'column', alignItems: msg.role === 'user' ? 'flex-end' : 'flex-start' }}>
                <ChatBubble role={msg.role}>
                  <strong>{msg.role === 'user' ? 'You' : 'Jetty'}:</strong> {msg.content}
                </ChatBubble>
                <Small style={{ margin: msg.role === 'user' ? '0 0.5rem 0 0' : '0 0 0 0.5rem', alignSelf: msg.role === 'user' ? 'flex-end' : 'flex-start' }}>{new Date(msg.timestamp).toLocaleTimeString()}</Small>
              </div>
            ))}
          </ChatContainer>
          <InputRow>
            <FuturisticInput
              type="text"
              value={userInput}
              onChange={e => setUserInput(e.target.value)}
              placeholder="Type your message..."
              onKeyDown={e => { if (e.key === 'Enter') handleSend(); }}
            />
            <FuturisticButton onClick={handleSend}>Send</FuturisticButton>
          </InputRow>
          <InputRow>
            <FuturisticButton onClick={handleVoiceInput} disabled={!micEnabled || isListening}>🎤 Voice Input</FuturisticButton>
            <FuturisticButton onClick={handleToggleMic}>🔇 Toggle Mic</FuturisticButton>
            <MicStatus enabled={micEnabled}>{micStatus}</MicStatus>
          </InputRow>
          <InputRow>
            <FuturisticButton onClick={handleClearChat}>🗑️ Clear Chat</FuturisticButton>
            <FuturisticButton onClick={handleTestSystem}>🧪 Test System</FuturisticButton>
          </InputRow>
          <audio ref={audioRef} style={{ display: 'none' }} />
        </Card>
      </Main>
      <StatusBar>
        <div><NeonLabel>Messages:</NeonLabel> {messages.length}</div>
        <div>{isListening ? <span style={{ color: neonBlue }}>🎤 Listening...</span> : <span style={{ color: neonPurple }}>🔇 Not listening</span>}</div>
        <div><NeonLabel>Mic Status:</NeonLabel> {micStatus}</div>
      </StatusBar>
      <link href="https://fonts.googleapis.com/css2?family=Orbitron:wght@700&display=swap" rel="stylesheet" />
    </Container>
  );
}

export default App; 