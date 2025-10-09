from typing import Union
import Vertice

class Vertice:
  
  def __init__(self, index : int) -> None:
    """Inicializa um vértice com um índice único."""  
    self.index : int = index
    self.marcado : bool = False
    self.antecessor : int | Vertice = -1
    self.tempo_d : int = 0
    self.tempo_f : int = 0

  def __repr__(self) -> str: 
    """Retorna uma representação em string do vértice."""
    if self.marcado == True :
      marca = 'S'
    else: 
      marca = 'N'
    representacao = f"""ID: {self.index} 
                    \n"""
    return representacao
  
  def marcar(self) -> None:
    """Marca o vértice como visitado."""
    self.marcado = True
    
  def desmarcar(self) -> None:
    """Desmarca o vértice, indicando que não foi visitado."""
    self.marcado = False
  
  def esta_marcado(self) -> bool:
    """Verifica se o vértice está marcado (visitado)."""
    return self.marcado
  
  def set_antecessor(self, v) -> None:
    """Define o antecessor do vértice."""
    self.antecessor = v
  
  def get_antecessor(self) -> Union[int, Vertice]:
    """Retorna o antecessor do vértice."""
    return self.antecessor
  