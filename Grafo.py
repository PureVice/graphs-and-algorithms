# Tipo abstrato de dados Grafo
#   Operações necessárias:

# *   Criar grafo vazio com |V| vértices e nenhuma aresta
# *   Inserir aresta no grafo
# *   Verificar se uma aresta pertence ao grafo
# *   Obter lista de vértices adjacentes a um dado vértice
# *   Imprimir grafo
# *   Remover aresta do grafo
# *   Obter número de vértices e arestas
# *   Determinar se grafo é direcionado ou não

class Grafo:
  def __init__(self, direcionado=False):
    self.vertices = []
    self.arestas = []
    self.direcionado = direcionado
  def __repr__(self):

    return f'{self.vertices} \n {self.arestas}'

  def inserir_vertice(self, v):
    if v not in self.vertices:
      self.vertices.append(v)

  def inserir_aresta(self, u, v):
    if u in self.vertices and v in self.vertices:
      self.arestas.append((u, v))
      if not self.direcionado:
        self.arestas.append((v, u))

  def aresta_pertence(self, u, v):
    if u in self.vertices and v in self.vertices:
      return (u, v) in self.arestas or (v, u) in self.arestas
    return False

  def get_adjacentes(self, v):
    adjacentes = []
    
    if v in self.vertices:
      for u in self.vertices:
        if (u, v) in self.arestas or (v,u) in self.arestas:
          adjacentes.append((u))
    return adjacentes
  
