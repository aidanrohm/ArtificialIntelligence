#
# TTT game using search
# dml c 2019
#
#
#
import math
import random
import time
#
from tkinter import *
import tkinter.messagebox

#
# tttagent.search(state)  is the search method that will be used
import Question2.rohm_tttagent as rohm_tttagent

#
tk = Tk()
tk.title("Tic-Tac-Toe by Search")

#
# Class for TTT game
#
class TTTGame:
    #
    def __init__(self):
        self.board = [ [], [], [] ] # GUI game board
        self.state = [ [" "]*3, [" "]*3, [" "]*3 ] # search/state
        self.rsum= [0]*3  # row sums
        self.csum= [0]*3  # col sums
        self.rdsum=0      # right diagonal
        self.ldsum=0      # left diagonal
        self.nummoves = 0 # check for board full
        self.message=0    # message field on GUI
        self.makeBoard()  # Start the GUI

    #
    # keeps stats that makes telling a win/loss/draw easier
    def updateCounts(self,i,j,val):
        '''Updates the row/col sums for this move of X (val=1) or O (val==-1) to i,j'''
        self.rsum[i] += val
        self.csum[j] += val
        if i==j:
            self.rdsum +=val
        if i==2-j:
            self.ldsum +=val
        self.nummoves += 1
        return
    #
    def checkForWin(self):
        '''check the row/col counts for a win or draw, ret true, else ret false'''
        for i in range(0,3):
            if self.rsum[i]==3 or self.csum[i]==3 or self.ldsum==3 or self.rdsum==3:
                print("X wins")
                self.message["text"]="   X wins   "
                return True
            if self.rsum[i]==-3 or self.csum[i]==-3 or self.rdsum==-3 or self.ldsum==-3:
                print("O wins")
                self.message["text"]="   O wins   "
                return True
            if self.nummoves==9:
                print("Draw")
                self.message["text"]="   Draw   "
                return True
        return False
    #
    def onClick(self,button,loc):
        '''Button callback routine'''

        if button["text"]==" ":
            button["text"]="X"
            self.updateCounts(loc[0],loc[1],1)
            self.state[loc[0]][loc[1]]="X"
            self.message["text"]="      X moves      "
            if self.checkForWin() and self.nummoves<9:
                self.message["text"]="     X WINS    "
                tk.after(5000,tk.quit)
                return
            
            action,status = rohm_tttagent.mmsearch(self.state)
            
            if status and self.board[action[0]][action[1]]["text"]==" ":
                self.board[action[0]][action[1]]["text"]="O"
                self.updateCounts(action[0],action[1],-1)
                self.state[action[0]][action[1]]="O"
                self.message["text"]="      O moves      "
                if self.checkForWin() and self.nummoves<9:
                    self.message["text"]="     O WINS    "
                    tk.after(5000,tk.quit)
                    return
        else:
            print("You cannot move there")
            self.message["text"]="*You cannot Move there*"
        if self.nummoves>=9:
            self.message["text"]="     DRAW    "
            tk.after(5000,tk.quit)
            return

        return

    # Make the game board GUI
    def makeBoard(self):
        '''Use TK to make the game board'''
        self.message = Label( tk, text="X starts", font='Times 20 bold', bg='white',
                        fg='black', height=1, width=8)
        self.message.grid(row=2, column=0)

        button1 = Button(tk, text=" ", font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button1,(0,0)))
        button1.grid(row=3, column=0)
        self.board[0].append(button1)

        button2 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button2,(0,1)))
        button2.grid(row=3, column=1)
        self.board[0].append(button2)

        button3 = Button(tk, text=' ',font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button3,(0,2)))
        button3.grid(row=3, column=2)
        self. board[0].append(button3)

        button4 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button4,(1,0)))
        button4.grid(row=4, column=0)
        self.board[1].append(button4)

        button5 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button5,(1,1)))
        button5.grid(row=4, column=1)
        self.board[1].append(button5)

        button6 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button6,(1,2)))
        button6.grid(row=4, column=2)
        self.board[1].append(button6)

        button7 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button7,(2,0)))
        button7.grid(row=5, column=0)
        self.board[2].append(button7)

        button8 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button8,(2,1)))
        button8.grid(row=5, column=1)
        self.board[2].append(button8)

        button9 = Button(tk, text=' ', font='Times 20 bold', bg='gray',
                         fg='white', height=4, width=8, command=lambda: self.onClick(button9,(2,2)))
        button9.grid(row=5, column=2)
        self.board[2].append(button9)

    def run(self):
        tk.mainloop()

        #---------------------------

if __name__ == "__main__":
    mygame = TTTGame()
    mygame.run()
    print("Average total nodes expanded per move in game: ",sum(rohm_tttagent.mx.gNodesList)/len(rohm_tttagent.mx.gNodesList))

