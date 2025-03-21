import React, { useState, useEffect } from "react";
import "./TicTacToe.css";
import { Button } from "@chakra-ui/react";
import { useDisclosure } from "@chakra-ui/react";
import { Spinner } from "@chakra-ui/react";
import { Input } from "@chakra-ui/react";

import {
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalBody,
  ModalCloseButton,
} from "@chakra-ui/react";

import { useToast } from "@chakra-ui/react";
import VisualLearningModal from "./VisualLearningModal";
import AdditionMethodModal from "./AdditionMethodModal";

const Cell = ({ num, onClick: onCellClick, cells, question }) => {
  const cellValue = cells[num];
  const cellClassName = cellValue ? `cell cell-${cellValue}` : "cell";

  return (
    <td
      className={cellClassName}
      onClick={() => onCellClick(num, question.num1, question.num2)}
    >
      {cellValue ? (
        cellValue
      ) : (
        <div className="question-display">
          {" "}
          {`${question?.num1} X ${question?.num2}`}
        </div>
      )}
    </td>
  );
};

const TicTacToe = () => {
  const [turn, setTurn] = useState("X");
  const [cells, setCells] = useState(Array(9).fill(""));
  const [winner, setWinner] = useState();
  const [isDraw, setIsDraw] = useState(false);
  const [questions, setQuestions] = useState([]);
  const [isLoading, setIsLoading] = useState(true);
  const [isError, setIsError] = useState(false);
  const [modalInfo, setModalInfo] = useState({});
  const [currAns, setCurrAns] = useState("");

  const { isOpen, onOpen, onClose } = useDisclosure();
  const {
    isOpen: isVisualModalOpen,
    onOpen: onVisualModalOpen,
    onClose: onVisualModalClose,
  } = useDisclosure();
  const {
    isOpen: isAddModalOpen, onOpen: onAddModalOpen, onClose: onAddModalClose
  } = useDisclosure();

  const toast = useToast();

  const checkwinner = (arr) => {
    let combos = {
      across: [
        [0, 1, 2],
        [3, 4, 5],
        [6, 7, 8],
      ],
      down: [
        [0, 3, 6],
        [1, 4, 7],
        [2, 5, 8],
      ],
      diagonal: [
        [0, 4, 8],
        [2, 4, 6],
      ],
    };
    for (let combo in combos) {
      combos[combo].forEach((pattern) => {
        if (
          arr[pattern[0]] === "" ||
          arr[pattern[1]] === "" ||
          arr[pattern[2]] === ""
        ) {
        } else if (
          arr[pattern[0]] === arr[pattern[1]] &&
          arr[pattern[1]] === arr[pattern[2]]
        ) {
          setWinner(arr[pattern[0]]);
        }
      });
    }
  };

  const handleCellClick = (cellNum, num1, num2) => {
    if (winner || cells[cellNum] !== "") return;

    setModalInfo({ cellNum, num1, num2 });
    onOpen();
  };

  const handleCellClaim = (cellNum) => {
    let arr = [...cells];
    if (turn === "X") {
      arr[cellNum] = "X";
      setTurn("O");
    } else {
      arr[cellNum] = "O";
      setTurn("X");
    }
    checkwinner(arr);
    setCells(arr);
    if (!arr.includes("") && !winner) {
      setIsDraw(true);
    }
  };

  const handleReset = () => {
    setWinner();
    setIsDraw(false);
    setCells(Array(9).fill(""));

    setTurn("X");

    setIsDraw(false);
    setQuestions([]);
    fetchQuestions();
    setModalInfo({});
  };

  const handleSubmitAns = (num1, num2, currAns) => {
    if (!currAns) return;

    currAns = parseInt(currAns);
    const isCorrect = num1 * num2 === currAns;
    console.log(isCorrect);

    if (isCorrect) {
      toast({
        title: "Success!",
        description: "Your answer is correct. You have claimed this box.",
        status: "success",
        duration: 5000,
        isClosable: true,
        position: "top-right",
      });

      handleModalClose();
      handleCellClaim(modalInfo.cellNum);
    } else {
      toast({
        title: "Incorrect Answer",
        description: "Please try again.",
        status: "error",
        duration: 5000,
        isClosable: true,
        position: "top-right",
      });
    }
  };

  const handleModalClose = () => {
    onClose();
    setCurrAns("");
  };

  const handleVisualModalOpen = () => {
    onVisualModalOpen();
  };

  const handleAddModalOpen = () => {
    onAddModalOpen();
  };


  const fetchQuestions = async () => {
    try {
      if (isError) setIsError(false);
      if (!isLoading) setIsLoading(true);

      const response = await fetch(
        "http://127.0.0.1:8000/api/generate-questions/"
      );
      const data = await response.json();

      console.log(data.questions);
      setQuestions(data.questions);
      setIsLoading(false);
    } catch (error) {
      console.error("Error fetching questions:", error);
      setIsError(true);
      setIsLoading(false);
    }
  };

  useEffect(() => {
    fetchQuestions();
  }, []);

  if (isLoading)
    return (
      <div className="TicTacToe__container TicTacToe__loader">
        <Spinner color="teal.500" size="xl" />
      </div>
    );

  if (isError) return <div>Error fetching questions!</div>;

  return (
    <div className="TicTacToe__container">
      <div className={`winner ${winner || isDraw ? "show" : ""}`}>
        {winner ? `Winner is: ${winner}` : isDraw ? "Its a draw" : ""}
      </div>
      <table>
        <tbody>
          <tr>
            <Cell
              num={0}
              onClick={handleCellClick}
              cells={cells}
              question={questions[0]}
            />
            <Cell
              num={1}
              onClick={handleCellClick}
              cells={cells}
              question={questions[1]}
            />
            <Cell
              num={2}
              onClick={handleCellClick}
              cells={cells}
              question={questions[2]}
            />
          </tr>
          <tr>
            <Cell
              num={3}
              onClick={handleCellClick}
              cells={cells}
              question={questions[3]}
            />
            <Cell
              num={4}
              onClick={handleCellClick}
              cells={cells}
              question={questions[4]}
            />
            <Cell
              num={5}
              onClick={handleCellClick}
              cells={cells}
              question={questions[5]}
            />
          </tr>
          <tr>
            <Cell
              num={6}
              onClick={handleCellClick}
              cells={cells}
              question={questions[6]}
            />
            <Cell
              num={7}
              onClick={handleCellClick}
              cells={cells}
              question={questions[7]}
            />
            <Cell
              num={8}
              onClick={handleCellClick}
              cells={cells}
              question={questions[8]}
            />
          </tr>
        </tbody>
      </table>

      <Button
        className="reset-button"
        colorScheme="teal"
        onClick={handleReset}
        size="lg"
      >
        Reset
      </Button>

      <Modal isOpen={isOpen} onClose={handleModalClose}>
        <ModalOverlay />
        <ModalContent>
          <ModalHeader>Solve this question to claim the box</ModalHeader>
          <ModalCloseButton />
          <ModalBody>
            <div className="Modal__question-display">
              {`${modalInfo.num1} X ${modalInfo.num2}`}
              <span> = </span>
              <Input
                value={currAns}
                onChange={(e) => setCurrAns(e.target.value)}
              />
            </div>
            <div className="Modal__ansbtn">
              <Button
                colorScheme="teal"
                onClick={() =>
                  handleSubmitAns(modalInfo.num1, modalInfo.num2, currAns)
                }
              >
                Submit Answer
              </Button>
            </div>

            <div className="Modal__learningbtns">
              <Button
                colorScheme="orange"
                variant={"outline"}
                onClick={handleVisualModalOpen}
              >
                Visual Learning
              </Button>
              <Button colorScheme="pink" variant={"outline"} onClick={handleAddModalOpen}>
                Addition Method
              </Button>
            </div>
          </ModalBody>
        </ModalContent>
      </Modal>

      <VisualLearningModal
        isOpen={isVisualModalOpen}
        onOpen={onVisualModalOpen}
        onClose={onVisualModalClose}
        num1={modalInfo.num1}
        num2={modalInfo.num2}
      />

      <AdditionMethodModal 
        isOpen={isAddModalOpen}
        onOpen={onAddModalOpen}
        onClose={onAddModalClose}
        num1={modalInfo.num1}
        num2={modalInfo.num2} 
      />
    </div>
  );
};

export default TicTacToe;
