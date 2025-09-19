# Tipo abstrato de dados Grafo
#   Operações necessárias:

# *   Criar grafo vazio com |V| vértices e nenhuma aresta - check
# *   Inserir aresta no grafo - check
# *   Verificar se uma aresta pertence ao grafo - check
# *   Obter lista de vértices adjacentes a um dado vértice - check
# *   Imprimir grafo - check ?
# *   Remover aresta do grafo
# *   Obter número de vértices e arestas
# *   Determinar se grafo é direcionado ou não

class Grafo:
  
  def __init__(self, direcionado=False):
    
    self.lista_adjacencia = {}  
    self.direcionado = direcionado
    self._num_arestas = 0

  def __repr__(self):
    
    if not self.lista_adjacencia:
        return "Grafo(V=0, E=0)"
    
    repr_str = ""
    for v, vizinhos in self.lista_adjacencia.items():
        repr_str += f"  {v} -> {sorted(list(vizinhos))}\n"
    return f"Grafo(V={self.get_numero_vertices()}, E={self.get_numero_arestas()}):\n{repr_str.strip()}"

  def get_numero_vertices(self):
    return len(self.lista_adjacencia)

  def get_numero_arestas(self):
    return self._num_arestas

  def get_vertices(self):
    return list(self.lista_adjacencia.keys())

  def inserir_vertice(self, v):
    if v not in self.lista_adjacencia:
        self.lista_adjacencia[v] = set()

  def inserir_aresta(self, u, v):
    
    if v not in self.lista_adjacencia[u]:
        self.lista_adjacencia[u].add(v)
        if not self.direcionado:
            self.lista_adjacencia[v].add(u)
        self._num_arestas += 1

  def remover_aresta(self, u, v):
    if self.aresta_pertence(u, v):
        self.lista_adjacencia[u].remove(v)
        if not self.direcionado:
            self.lista_adjacencia[v].remove(u)
        self._num_arestas -= 1

  def aresta_pertence(self, u, v):
    if u not in self.lista_adjacencia:
        return False
    return v in self.lista_adjacencia[u]

  def get_adjacentes(self, v):
    return list(self.lista_adjacencia.get(v, set()))

  def is_direcionado(self):
    return self.direcionado