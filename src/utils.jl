# Miscellaneous utilities

"""
    lazy_collect(collection)
    lazy_collect(element_type, collection)

Lazily `collect` an iterable. Returns the original `collection` if it is already
an `Array` of the appropriate type.
"""
lazy_collect(collection) = collect(collection)
lazy_collect(collection::Array) = collection
lazy_collect(element_type::Type, collection) = collect(element_type, collection)
lazy_collect(::Type{T}, collection::Array{T}) where {T} = collection
