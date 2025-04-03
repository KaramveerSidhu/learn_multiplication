import './App.css';
import TicTacToe from './components/TicTacToe';
import WelcomeScreen from './components/WelcomeScreen';
import React, { useState } from 'react';

function App() {
  const [isGameStarted, setIsGameStarted] = useState(false);

  return (
    <div className="App">
      {isGameStarted ? (
        <TicTacToe />
      ) : (
        <WelcomeScreen onStart={() => setIsGameStarted(true)} />
      )}
    </div>
  );
}

export default App;
