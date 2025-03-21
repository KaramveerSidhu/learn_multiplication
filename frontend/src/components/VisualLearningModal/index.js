import React from "react";
import {
  Modal,
  ModalOverlay,
  ModalContent,
  ModalHeader,
  ModalBody,
  ModalCloseButton,
  useDisclosure,
} from "@chakra-ui/react";
import "./VisualLearningModal.css";

const VisualLearningModal = ({ isOpen, onOpen, onClose, num1, num2 }) => {
  const handleModalClose = () => {
    onClose();
  };

  return (
    <Modal size={"2xl"} isOpen={isOpen} onClose={handleModalClose}>
      <ModalOverlay />
      <ModalContent>
        <ModalCloseButton />
        <ModalBody className="VisualLearningModalModal__container">
          <div className="VisualLearningModalModal">
            <div className="VisualLearningModalModal__question-display">
              {`${num1} X ${num2}`}
            </div>

            <div className="VisualLearningModalModal__top">
              <div className="VisualLearningModalModal__block VisualLearningModalModal__hidden-block">
                {" "}
              </div>
              <div className="VisualLearningModalModal__parentBlockRow">
                {" "}
                {new Array(num2).fill("").map(() => {
                  return (
                    <div className="VisualLearningModalModal__block"></div>
                  );
                })}{" "}
              </div>
            </div>

            <div className="VisualLearningModalModal__parentMain">
              <div className="VisualLearningModalModal__parentBlockCol">
                {" "}
                {new Array(num2).fill("").map(() => {
                  return (
                    <div className="VisualLearningModalModal__block"></div>
                  );
                })}{" "}
              </div>

              <div>
                {new Array(num1).fill("").map((_, idx) => {
                  return (
                    <div
                      className={`${
                        idx === 0
                          ? "VisualLearningModalModal__parentBlockRow VisualLearningModalModal__parentBlockRowMain VisualLearningModalModal__parentBlockRowFirst"
                          : "VisualLearningModalModal__parentBlockRow VisualLearningModalModal__parentBlockRowMain"
                      }`}
                    >
                      {" "}
                      {new Array(num2).fill("").map(() => {
                        return (
                          <div className="VisualLearningModalModal__block"></div>
                        );
                      })}{" "}
                    </div>
                  );
                })}
              </div>
            </div>
          </div>
        </ModalBody>
      </ModalContent>
    </Modal>
  );
};

export default VisualLearningModal;
