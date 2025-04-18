const wordToNumberMap = {
  zero: 0,
  one: 1,
  two: 2,
  three: 3,
  four: 4,
  five: 5,
  six: 6,
  seven: 7,
  eight: 8,
  nine: 9,
  ten: 10,
  eleven: 11,
  twelve: 12,
  thirteen: 13,
  fourteen: 14,
  fifteen: 15,
  sixteen: 16,
  seventeen: 17,
  eighteen: 18,
  nineteen: 19,
  twenty: 20,
  thirty: 30,
  forty: 40,
  fifty: 50,
  sixty: 60,
  seventy: 70,
  eighty: 80,
  ninety: 90,
  hundred: 100,
  thousand: 1000,
};

export const parseSpeechToNumber = (speech) => {
  const words = speech.toLowerCase().split(" ");
  let number = 0;
  let current = 0;
  let invalid = false;

  console.log("Input Speech: ", speech);

  words.forEach((word) => {
    if (!isNaN(word)) {
      current += parseInt(word, 10);
    } else if (wordToNumberMap[word] !== undefined) {
      const value = wordToNumberMap[word];
      if (value === 100 || value === 1000) {
        current *= value; 
      } else {
        current += value; 
      }
    } else {
      console.error(`Unrecognized word: ${word}`);
      invalid = true
    }
  });

  if(invalid) return undefined

  number += current;

  console.log("Parsed Number: ", number);
  return number;
};
