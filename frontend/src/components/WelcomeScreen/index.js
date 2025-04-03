import React from "react";
import "./WelcomeScreen.css";

import { useEffect, useRef } from "react";

const WelcomeScreen = ({ onStart }) => {
    const hasRun = useRef(false);

    useEffect(() => {
        if (hasRun.current) return;
        hasRun.current = true;

        // fetch(
        // `${process.env.REACT_APP_API_SERVER}/api/welcome`
        // );
    }
    , []);

  return (
    <div className="welcome-body">
      <h1 className="welcome-h1">Welcome to Tic Tac Maths!</h1>
      <div className="welcome-instructions">
        <p className="welcome-instruction-item">1. The game is played on a Tic Tac Toe board.</p>
        <p className="welcome-instruction-item">2. To claim a box, you must solve a multiplication question correctly.</p>
        <p className="welcome-instruction-item">3. The first player to get three boxes in a row (horizontally, vertically, or diagonally) wins!</p>
        <p className="welcome-instruction-item">4. Have fun and sharpen your multiplication skills!</p>
      </div>
      <button className="welcome-start-button" onClick={onStart}>
        Start Game
      </button>
    </div>
  );
};

export default WelcomeScreen;
