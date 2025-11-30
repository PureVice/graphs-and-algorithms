Grafos e Algoritmos

Este repositório contém uma implementação robusta de estruturas de dados de grafos e algoritmos clássicos em Python. O projeto foi desenhado com foco em clareza, utilizando type hints e separação de responsabilidades em classes distintas para grafos simples, ponderados e com fluxo.

📋 Funcionalidades

O projeto cobre uma ampla gama de algoritmos de grafos, divididos por categorias:

Estruturas de Dados

    Grafo: Representação básica (direcionado ou não).

    GrafoPonderado: Extensão para arestas com pesos.

    GrafoComFluxo: Extensão para redes de fluxo (capacidade e fluxo).

    DisjointSet: Estrutura de dados para conjuntos disjuntos (Union-Find), usada no algoritmo de Kruskal.

Algoritmos de Busca e Travessia

    BFS (Breadth-First Search): Busca em largura.

    DFS (Depth-First Search): Busca em profundidade (versões iterativa e recursiva).

    DFS com Tempo: Registra tempos de descoberta e finalização (útil para ordenação topológica e SCC).

Caminho Mínimo

    Dijkstra: Para grafos com pesos não negativos (usa Min-Heap).

    Bellman-Ford: Para grafos com pesos que podem ser negativos (detecta ciclos negativos).

Árvore Geradora Mínima (MST)

    Prim: Implementação eficiente com Heap.

    Kruskal: Utiliza a estrutura DisjointSet.

Conectividade

    Kosaraju-Sharir: Para encontrar Componentes Fortemente Conexos (SCC) em grafos direcionados.

Fluxo em Redes

    Ford-Fulkerson: Cálculo de fluxo máximo utilizando caminhos aumentantes.

    Capacity Scaling: Variação otimizada do Ford-Fulkerson para melhor desempenho.

📂 Estrutura do Projeto

    Grafo.py: Arquivo principal contendo as classes Grafo, GrafoPonderado e GrafoComFluxo.

    Vertice.py: Define a classe Vertice, que armazena metadados como cor, tempos de visita, antecessores e rótulos.

    DisjointSet.py: Implementação auxiliar para o algoritmo de Kruskal.

    Main.py: Arquivo de exemplo demonstrando a instanciação de grafos e execução dos algoritmos.

🚀 Como Usar

Pré-requisitos

    Python 3.10 ou superior (devido ao uso de type hints como int | Vertice).

Instalação

Clone o repositório:
Bash

git clone https://github.com/purevice/graphs-and-algorithms.git
cd graphs-and-algorithms

Exemplo de Uso

Aqui está um exemplo básico de como criar um grafo ponderado e executar o algoritmo de Dijkstra, baseado no arquivo Main.py:
Python

from Grafo import GrafoPonderado

# 1. Instanciar o grafo (direcionado)
g = GrafoPonderado(direcionado=True)

# 2. Inserir arestas (Origem, Destino, Peso)
# O vértice é criado automaticamente se não existir
g.inserir_aresta('s', 't', 10)
g.inserir_aresta('s', 'y', 5)
g.inserir_aresta('t', 'y', 2)
g.inserir_aresta('t', 'x', 1)
g.inserir_aresta('x', 'z', 4)

# 3. Executar algoritmos
print("--- Dijkstra a partir de 's' ---")
g.djikstra('s')

# Reconstruir caminho até um vértice específico
caminho = g.reconstruir_caminho('s', 'z')
print(f"Caminho de 's' até 'z': {caminho}")

# 4. Verificar Árvore Geradora Mínima (Kruskal)
mst = g.kruskal()
print("\nMST (Kruskal):", mst)

🧪 Testes

Você pode rodar o arquivo Main.py para ver os algoritmos em ação com exemplos pré-definidos:
Bash

python Main.py

🛠 Tecnologias

    Linguagem: Python

    Bibliotecas: heapq (para filas de prioridade), collections (deque), typing.

Desenvolvido como parte de estudos em Teoria dos Grafos e Algoritmos
