
# %%
from Grafo import Grafo

# %% [markdown]
# Montagem de um grafo arbitrário
# 

# %%
g = Grafo()
lista = [0, 1, 2, 3, 4, 5, 6, 7]
g.inserir_lista_vertices(lista)



# %%
g

# %%
g.inserir_aresta(1, 5)
g.inserir_aresta(2, 4)
g.inserir_aresta(2, 6)
g.inserir_aresta(3, 5)
g.inserir_aresta(3, 7)
g.inserir_aresta(4, 7)
g.inserir_aresta(5, 6)
g.inserir_aresta(6, 7)


# %%
num_arestas = g.get_numero_arestas()
num_vertices = g.get_numero_vertices()

print(f"arestas: {num_arestas} \nvertices: {num_vertices}")

# %%
g

# %%
#g.dfs()

g.dfs_iterativa(3)
# %%

g.imprimir_caminho(1, 4)



# %%
